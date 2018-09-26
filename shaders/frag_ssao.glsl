#version 420
#extension GL_EXT_gpu_shader4 : enable

in vec2 outUV;

layout(location = 0)out vec4 def_AO;

uniform sampler2D tex0;//color
uniform sampler2D tex1;//normals
uniform sampler2D tex2;//orig pos

uniform sampler2D noiseTex;//noise

uniform sampler2D depthTex;

uniform vec2 pixelSize;//in texture space

uniform mat4 pMatrix;
uniform mat4 mvpMatrix;

float LinearizeDepth(float z){
		const float zNear = 5.0; // camera z near
		const float zFar = 10000.0; // camera z far

		return (2.0 * zNear) / (zFar + zNear - z * (zFar - zNear));
}//

/*----------------------------------------------------------------------------*/
//	ssao uniforms:
const int MAX_KERNEL_SIZE = 128;
uniform int uKernelSize=16;//16
uniform vec3 uKernelOffsets[MAX_KERNEL_SIZE];
uniform float uRadius = 20.0;//1.5
uniform float uPower = 1.0;//2.0

float linearizeDepth(in float depth, in mat4 projMatrix) {
	return projMatrix[3][2] / (depth - projMatrix[2][2]);
}

float ssao(in mat3 kernelBasis, in vec3 originPos, in float radius, in vec3 normal) {
	float occlusion = 0.0;
	
	vec4 originPosProj = mvpMatrix * vec4(originPos, 1.0);//for rangecheck
	originPosProj.xyz /= originPosProj.w; 
	originPosProj.xyz = originPosProj.xyz * 0.5 + 0.5; // scale/bias to texcoords
	float originDepth = texture(depthTex, originPosProj.xy).r;

	for (int i = 0; i < uKernelSize; ++i) {
		//	get sample position:
		vec3 samplePos = kernelBasis * uKernelOffsets[i];
		samplePos = samplePos * radius + originPos;
		
		//samplePos = originPos + uKernelOffsets[i];
		//	project sample position:
		vec4 offset = mvpMatrix * vec4(samplePos, 1.0);
		
		offset.xyz /= offset.w; // only need xy
		offset.xyz = offset.xyz * 0.5 + 0.5; // scale/bias to texcoords

		// get sample depth:
		float sampleDepth = texture(depthTex, offset.xy).r;

		// get sample normal
		vec3 sampleNormal = texture(tex1, offset.xy).rgb;
		sampleNormal = normalize(sampleNormal);

		float offsetDepth = offset.z;

		float rangeCheck = 1.0f;// abs(offsetDepth - sampleDepth) < radius ? 1.0 : 0.0;

		if (dot(normal, sampleNormal) < 0.9) {
			occlusion += (sampleDepth <= offsetDepth ? 1.0 : 0.0)*rangeCheck;// *rangeCheck;
		}
	}

	occlusion = 1.0 - (occlusion / float(uKernelSize));
	return pow(occlusion, uPower);
}

void main(){
	/////////////////
	// SECOND PASS

	vec2 coord = outUV.xy;
	
	vec3 normal = texture(tex1, coord).rgb;
	if (length(normal) < 0.1) {
		def_AO.rgb = vec3(1.0);//sky
		return;
	}
	normal = normalize(normal);
	float originDepth = texture(depthTex, coord).r;

	originDepth = linearizeDepth(originDepth, pMatrix);
	vec3 originPos = texture(tex2, coord).rgb;// texture(tex2, coord).rgb;
	
	const float uNoiseScale = 4.0;
	vec3 rvec = texture(noiseTex, coord*uNoiseScale).rgb;// *2 - 1;
	rvec = normalize(rvec);

	vec3 tangent = normalize(rvec - normal * dot(rvec, normal));
	vec3 bitangent = cross(tangent, normal);
	mat3 kernelBasis = mat3(tangent, bitangent, normal);

	float ssaoVal = ssao(kernelBasis, originPos, uRadius, normal);
	def_AO.rgb = vec3(ssaoVal);
	return;
}

