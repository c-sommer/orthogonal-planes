// This software is in the public domain. Where that dedication is not
// recognized, you are granted a perpetual, irrevocable license to copy,
// distribute, and modify this file as you see fit.
// https://github.com/ddiakopoulos/tinyply
// Version 2.2

#include <vector>
#include <sstream>
#include <fstream>
#include <iostream>
#include <Eigen/Dense>

#include <tinyply/tinyply.h>

void read_ply_file(const std::string & filepath, std::vector<Eigen::Vector3f>& pts, std::vector<Eigen::Vector3f>& ns)
{
	try {
	
		std::ifstream ss(filepath, std::ios::binary);
		if (ss.fail())
		    throw std::runtime_error("failed to open " + filepath);

		tinyply::PlyFile file;
		file.parse_header(ss);

		// Tinyply treats parsed data as untyped byte buffers. See below for examples.
		std::shared_ptr<tinyply::PlyData> vertices, normals;

		// The header information can be used to programmatically extract properties on elements
		// known to exist in the header prior to reading the data. For brevity of this sample, properties 
		// like vertex position are hard-coded: 
		try {
		    vertices = file.request_properties_from_element("vertex", { "x", "y", "z" });
		} catch (const std::exception & e) {
		    std::cerr << "tinyply exception: " << e.what() << std::endl;
		}

		try {
		    normals = file.request_properties_from_element("vertex", { "nx", "ny", "nz" });
		} catch (const std::exception & e) {
		    std::cerr << "tinyply exception: " << e.what() << std::endl;
		}

		file.read(ss);

		// type casting to your own native types
		pts = std::vector<Eigen::Vector3f>(vertices->count);
		std::memcpy(pts.data(), vertices->buffer.get(), vertices->buffer.size_bytes());
		ns = std::vector<Eigen::Vector3f>(normals->count);
		std::memcpy(ns.data(), normals->buffer.get(), normals->buffer.size_bytes());
		
	} catch (const std::exception & e) {
	
		std::cerr << "Caught tinyply exception: " << e.what() << std::endl;
		
	}
}
