#include "opencv2/opencv.hpp"

using namespace cv;

void creat_xml(Mat matrix)
{
	FileStorage fs("matrix/matrix.xml", FileStorage::WRITE);
	fs << "matrix.xml" << img;
	fs.release();
}

int main()
{
	return 0;
}
