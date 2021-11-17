#include <iostream>
#include <fstream>
#include <array>

#include "Eigen.h"
#include "VirtualSensor.h"

#define IDX(x, y, w) ((y) * w) + (x)

struct Vertex
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		// position stored as 4 floats (4th component is supposed to be 1.0)
		Vector4f position;
	// color stored as 4 unsigned char
	Vector4uc color;
};

static inline bool IsPointValid(const Vertex& point)
{
	return point.position.x() != MINF && point.position.y() != MINF && point.position.z() != MINF;
}

static inline bool IsTriangleValid(const Vertex& test1, const Vertex& test2, const Vertex& test3, float threshold)
{
	return (IsPointValid(test1) && IsPointValid(test2) && IsPointValid(test3)) &&
		(test1.position - test2.position).norm() <= threshold &&
		(test1.position - test3.position).norm() <= threshold &&
		(test2.position - test3.position).norm() <= threshold;
}

static std::ofstream& operator<< (std::ofstream& out, const Vertex& point)
{
	bool pointValid = IsPointValid(point);
	out << (pointValid ? point.position.x() : 0.0f) << " "
		<< (pointValid ? point.position.y() : 0.0f) << " "
		<< (pointValid ? point.position.z() : 0.0f) << " "
		<< static_cast<int>(point.color[0]) << " "
		<< static_cast<int>(point.color[1]) << " "
		<< static_cast<int>(point.color[2]) << " "
		<< static_cast<int>(point.color[3]) << std::endl;
	return out;
}

static std::ofstream& operator<< (std::ofstream& out, std::array<uint32_t, 3> tri)
{
	out << 3 << " " << tri[0] << " " << tri[1] << " " << tri[2] << std::endl;
	return out;
}

bool WriteMesh(Vertex* vertices, unsigned int width, unsigned int height, const std::string& filename)
{
	float edgeThreshold = 0.01f; // 1cm

	// TODO 2: use the OFF file format to save the vertices grid (http://www.geomview.org/docs/html/OFF.html)
	// - have a look at the "off_sample.off" file to see how to store the vertices and triangles
	// - for debugging we recommend to first only write out the vertices (set the number of faces to zero)
	// - for simplicity write every vertex to file, even if it is not valid (position.x() == MINF) (note that all vertices in the off file have to be valid, thus, if a point is not valid write out a dummy point like (0,0,0))
	// - use a simple triangulation exploiting the grid structure (neighboring vertices build a triangle, two triangles per grid cell)
	// - you can use an arbitrary triangulation of the cells, but make sure that the triangles are consistently oriented
	// - only write triangles with valid vertices and an edge length smaller then edgeThreshold

	// TODO: Get number of vertices
	unsigned int nVertices = width * height;

	// TODO: Determine number of valid faces
	unsigned nFaces = 0;
	std::vector<std::array<uint32_t, 3>> faces;

	for (uint32_t y = 0; y < (height - 1); ++y)
	{
		for (uint32_t x = 0; x < (width - 1); ++x)
		{
			uint32_t tl = IDX(x, y, width);
			uint32_t tr = IDX(x + 1, y, width);
			uint32_t bl = IDX(x, y + 1, width);
			uint32_t br = IDX(x + 1, y + 1, width);

			if(IsTriangleValid(vertices[tl], vertices[tr], vertices[bl], edgeThreshold))
			{
				faces.push_back({ tl, bl, tr });
				nFaces++;
			}
			if(IsTriangleValid(vertices[tr], vertices[bl], vertices[br], edgeThreshold))
			{
				faces.push_back({ bl, tr, br });
				nFaces++;
			}
		}
	}

	// Write off file
	std::ofstream outFile(filename);
	if (!outFile.is_open()) return false;

	// write header
	outFile << "COFF" << std::endl;

	outFile << "# numVertices numFaces numEdges" << std::endl;

	outFile << nVertices << " " << nFaces << " 0" << std::endl;

	// TODO: save vertices
	for(uint32_t index = 0; index < nVertices; ++index)
	{
		outFile << vertices[index];
	}

	// TODO: save valid faces
	for(auto& face : faces)
	{
		outFile << face;
	}

	// close file
	outFile.close();

	return true;
}

int main()
{
	// Make sure this path points to the data folder
	std::string filenameIn = "../Data/";
	std::string filenameBaseOut = "mesh_";

	// load video
	std::cout << "Initialize virtual sensor..." << std::endl;
	VirtualSensor sensor;
	if (!sensor.Init(filenameIn))
	{
		std::cout << "Failed to initialize the sensor!\nCheck file path!" << std::endl;
		return -1;
	}

	// convert video to meshes
	while (sensor.ProcessNextFrame())
	{
		// get ptr to the current depth frame
		// depth is stored in row major (get dimensions via sensor.GetDepthImageWidth() / GetDepthImageHeight())
		float* depthMap = sensor.GetDepth();
		// get ptr to the current color frame
		// color is stored as RGBX in row major (4 byte values per pixel, get dimensions via sensor.GetColorImageWidth() / GetColorImageHeight())
		BYTE* colorMap = sensor.GetColorRGBX();

		// get depth intrinsics
		Matrix3f depthIntrinsics = sensor.GetDepthIntrinsics();
		Matrix3f depthIntrinsicsInv = depthIntrinsics.inverse();

		float fX = depthIntrinsics(0, 0);
		float fY = depthIntrinsics(1, 1);
		float cX = depthIntrinsics(0, 2);
		float cY = depthIntrinsics(1, 2);

		// compute inverse depth extrinsics
		Matrix4f depthExtrinsicsInv = sensor.GetDepthExtrinsics().inverse();

		Matrix4f trajectory = sensor.GetTrajectory();
		Matrix4f trajectoryInv = sensor.GetTrajectory().inverse();

		// TODO 1: back-projection
		// write result to the vertices array below, keep pixel ordering!
		// if the depth value at idx is invalid (MINF) write the following values to the vertices array
		// vertices[idx].position = Vector4f(MINF, MINF, MINF, MINF);
		// vertices[idx].color = Vector4uc(0,0,0,0);
		// otherwise apply back-projection and transform the vertex to world space, use the corresponding color from the colormap
		Vertex* vertices = new Vertex[sensor.GetDepthImageWidth() * sensor.GetDepthImageHeight()];
		uint32_t w = sensor.GetDepthImageWidth();
		uint32_t h = sensor.GetDepthImageHeight();

		for (uint32_t y = 0; y < h; ++y)
		{
			for (uint32_t x = 0; x < w; ++x)
			{
				uint32_t index = IDX(x, y, w);
				float depth = depthMap[index];

				if (depth == MINF)
				{
					vertices[index].position = Vector4f::Constant(MINF);
					vertices[index].color = Vector4uc::Constant(0);
				}
				else
				{
					Vector3f projSpace = Vector3f(x * depth, y * depth, depth);
					Vector4f camSpace = Vector4f::Ones();
					camSpace.block(0, 0, 3, 1) = depthIntrinsicsInv * projSpace;
					Vector4f worldSpace = trajectoryInv * depthExtrinsicsInv * camSpace;
					Map<const Vector4uc> color(&colorMap[index * 4u]);

					vertices[index].position = worldSpace;
					vertices[index].color = color;
				}
			}
		}

		// write mesh file
		std::stringstream ss;
		ss << filenameBaseOut << sensor.GetCurrentFrameCnt() << ".off";
		if (!WriteMesh(vertices, sensor.GetDepthImageWidth(), sensor.GetDepthImageHeight(), ss.str()))
		{
			std::cout << "Failed to write mesh!\nCheck file path!" << std::endl;
			return -1;
		}

		// free mem
		delete[] vertices;
	}

	return 0;
}