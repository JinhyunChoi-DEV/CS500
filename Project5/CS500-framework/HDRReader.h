#pragma once

class HDRResult
{
public:
	int width, height;
	// each pixel takes 3 float32, each component can be of any value...
	float* cols;
};

class HDRReader {
public:
	static bool load(const char* fileName, HDRResult& res);
};