#include "VocabularyTree.h"
#include "opencv2/opencv.hpp"

#include <vector>
#include <set>
#include <string>
#include <sstream>
#include <fstream>
#include <chrono>

#include <fstream>

using namespace std;
using namespace cv;


int main() {
	try {
		vector<string> bookNames;
		vector<Mat> bookImgs;

		ifstream fin("/home/jetbot/catkin_ws/src/bitproject/ImageDB/train for demo/booknames.txt");
		if (!fin.is_open()) {
			cerr << "fail to open " << "booknames.txt" << endl;
			return 1;
		}

		string line;
		while (getline(fin, line))
		{
			cout << "test: " << line << endl;
			bookNames.push_back(line);
		}

		//========== �ӽ� ==========
		{
			string prefix = "/home/jetbot/catkin_ws/src/bitproject/ImageDB/train for demo/data";
			for (int i = 0; i < 12; i++) {
				stringstream ss;
				ss << prefix << setw(3) << setfill('0') << i << ".jpg";
				string filename = ss.str();
				Mat img = imread(filename, IMREAD_GRAYSCALE);
				bookImgs.push_back(img);
			}
		}
		//==========================

		// VocabularyTree ��ü ����
		VocabularyTree VTree(10, 3, 32); // K, L, dimension 

		chrono::system_clock::time_point set_start = std::chrono::system_clock::now();
		// ������ ����
		try {
			VTree.setBookImgs(bookImgs, bookNames);
		}
		catch (exception & e) {
			cerr << e.what() << endl;
			return 1;
		}

		chrono::system_clock::time_point set_end = std::chrono::system_clock::now();
		chrono::duration<double> setting_time = set_end - set_start;
		//cerr << "������ set �Ϸ�, �ɸ� �ð�: " << setting_time.count() << endl;

		// fitting ����
		cerr << "fitting ����" << endl;
		chrono::system_clock::time_point fit_start = std::chrono::system_clock::now();
		VTree.fit();
		chrono::system_clock::time_point fit_end = std::chrono::system_clock::now();
		chrono::duration<double> fitting_time = fit_end - fit_start;
		//cerr << "fitting �Ϸ�, �ɸ� �ð�: " << fitting_time.count() << "��" << endl;
				
		// �� ����
		VTree.saveToFile("/home/jetbot/catkin_ws/src/bitproject/ImageDB/imgdb_demo.vtree");
		cout << "fitting complete" << endl;

	}
	catch (exception & e) {
		cerr << e.what() << endl;
	}
	return 0;
}
