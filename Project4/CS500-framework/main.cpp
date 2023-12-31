///////////////////////////////////////////////////////////////////////
// Provides the framework a raytracer.
//
// Gary Herron
//
// Copyright 2012 DigiPen Institute of Technology
////////////////////////////////////////////////////////////////////////

#include <fstream>
#include <sstream>
#include <vector>
#include <string.h>
#include <ctime>

#ifdef _WIN32
	// Includes for Windows
#include <windows.h>
#include <cstdlib>
#include <limits>
#include <crtdbg.h>
#else
	// Includes for Linux
#include <stdlib.h>
#include <time.h> 
#endif

#include "geom.h"
#include "raytrace.h"

// Read a scene file by parsing each line as a command and calling
// scene->Command(...) with the results.
void ReadScene(const std::string inName, Scene* scene)
{
	std::ifstream input(inName.c_str());
	if (input.fail()) {
		std::cerr << "File not found: " << inName << std::endl;
		fflush(stderr);
		exit(-1);
	}

	// For each line in file
	for (std::string line; getline(input, line); ) {
		std::vector<std::string> strings;
		std::vector<float> floats;

		// Parse as parallel lists of strings and floats
		std::stringstream lineStream(line);
		for (std::string s; lineStream >> s; ) { // Parses space-separated strings until EOL
			float f;
			//std::stringstream(s) >> f; // Parses an initial float into f, or zero if illegal
			if (!(std::stringstream(s) >> f)) f = nan(""); // An alternate that produced NANs
			floats.push_back(f);
			strings.push_back(s);
		}

		if (strings.size() == 0) continue; // Skip blanks lines
		if (strings[0][0] == '#') continue; // Skip comment lines

		// Pass the line's data to Command(...)
		scene->Command(strings, floats);
	}

	input.close();
}

////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{
	time_t start, end;
	start = time(NULL);

	Scene* scene = new Scene();

	// Read the command line argument
	std::string inName = (argc > 1) ? argv[1] : "testscene.scn";
	std::string hdrName = inName;
	hdrName.replace(hdrName.size() - 3, hdrName.size(), "hdr");
	scene->hdrName = hdrName;

	// Read the scene, calling scene.Command for each line.
	ReadScene(inName, scene);

	scene->Finit();

	// Allocate and clear an image array
	Color* image = new Color[scene->width * scene->height];
	for (int y = 0; y < scene->height; y++)
		for (int x = 0; x < scene->width; x++)
			image[y * scene->width + x] = Color(0, 0, 0);

	// RayTrace the image
	scene->TraceImage(image, 4096);

	end = time(NULL);
	double result = (double)(end - start);
	std::cout << "---------------------" << std::endl;
	std::cout << "Taking Time: " << result << " Seconds" << std::endl;
}
