#include "voxel_model.h"

namespace voxel_model {
          
    void put_voxel(int x, int y, int z, std::vector<Point3d> &vertices, std::vector<Triangle> &faces) {
        
        auto v0 = Point3d(x - 0.5, y - 0.5, z - 0.5);
        auto v1 = Point3d(x + 0.5, y - 0.5, z - 0.5);
        auto v2 = Point3d(x - 0.5, y + 0.5, z - 0.5);
        auto v3 = Point3d(x + 0.5, y + 0.5, z - 0.5);
        auto v4 = Point3d(x - 0.5, y - 0.5, z + 0.5);
        auto v5 = Point3d(x + 0.5, y - 0.5, z + 0.5);
        auto v6 = Point3d(x - 0.5, y + 0.5, z + 0.5);
        auto v7 = Point3d(x + 0.5, y + 0.5, z + 0.5);
        int p = vertices.size();
        
        faces.push_back(Triangle(p+4, p+2, p+0));
        faces.push_back(Triangle(p+4, p+6, p+2));
        
        faces.push_back(Triangle(p+4, p+0, p+1));
        faces.push_back(Triangle(p+4, p+1, p+5));
        
        faces.push_back(Triangle(p+4, p+5, p+7));
        faces.push_back(Triangle(p+4, p+7, p+6));
        
        faces.push_back(Triangle(p+3, p+1, p+0));
        faces.push_back(Triangle(p+3, p+0, p+2));
        
        faces.push_back(Triangle(p+3, p+7, p+5));
        faces.push_back(Triangle(p+3, p+5, p+1));
        
        faces.push_back(Triangle(p+3, p+2, p+6));
        faces.push_back(Triangle(p+3, p+6, p+7));
        
        vertices.push_back(v0);
        vertices.push_back(v1);
        vertices.push_back(v2);
        vertices.push_back(v3);
        vertices.push_back(v4);
        vertices.push_back(v5);
        vertices.push_back(v6);
        vertices.push_back(v7);
        
    }
    
    int voxel_model(std::vector<cv::Mat_<uchar>> &voxel_data, std::string output_ply_filename, double offset_x, double offset_y, double offset_z, double scale) {
        
        std::cout << "Doing debug voxel model." << std::endl;
        std::cout << "x offset\t" << offset_x << std::endl;
        std::cout << "y offset\t" << offset_y << std::endl;
        std::cout << "z offset\t" << offset_z << std::endl;
        std::cout << "scale\t" << scale << std::endl;
        
        std::vector<Point3d> vertices;
        std::vector<Triangle> faces;
        
        uchar thresh=128;
        int nslices = voxel_data.size();
        for (int k=0; k<nslices; k++) {
            auto &mat = voxel_data[k];
            for (int i=0; i<mat.rows; i++) {
                for (int j=0; j<mat.cols; j++) {
                    
					if (k>0 && k<nslices && i>0 && i<mat.rows - 1 && j>0 && j<mat.cols - 1)
						if (voxel_data[k - 1](i, j) >= thresh && voxel_data[k + 1](i, j) >= thresh)
							if (voxel_data[k](i - 1, j) >= thresh && voxel_data[k](i + 1, j) >= thresh)
								if (voxel_data[k](i, j - 1) >= thresh && voxel_data[k](i, j + 1) >= thresh)
                                    continue;
                    
                    if (mat(i,j) >= thresh)
                        put_voxel(j, i, k, vertices, faces);
                }
            }
        }
        
        std::ofstream out(output_ply_filename, std::ios::binary);
        out << "ply" << std::endl;
        out << "format binary_little_endian 1.0" << std::endl;
        out << "element vertex " << vertices.size() << std::endl;
        out << "property float x" << std::endl;
        out << "property float y" << std::endl;
        out << "property float z" << std::endl;
        out << "element face " << faces.size() << std::endl;
        out << "property list uchar int vertex_indices" << std::endl;
        out << "end_header" << std::endl;
        
        // write vertices
        for (int i = 0; i < vertices.size(); i++) {
            
            // !!! note that transform does not include shift to image center !!!
            
            float x = vertices[i].x * scale + offset_x;
            float y = - vertices[i].y * scale + offset_y;
            float z = vertices[i].z * scale + offset_z;
            out.write((char*)&x, sizeof(float));
            out.write((char*)&y, sizeof(float));
            out.write((char*)&z, sizeof(float));
        }

        // write faces
        for (int i = 0; i < faces.size(); i++) {
            uchar s = 3;
            out.write((char*)&s, sizeof(uchar));            
            out.write((char*)&(faces[i].a), sizeof(int));
            out.write((char*)&(faces[i].b), sizeof(int));
            out.write((char*)&(faces[i].c), sizeof(int));
            
        }
        
        return 0;
    }
}
