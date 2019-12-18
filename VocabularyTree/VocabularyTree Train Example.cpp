#include "VocabularyTree.h"
#include "ImageDB.h"
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

		ifstream fin("booknames.txt");
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

		//========== 임시 ==========
		{
			string prefix = "img/train/data";
			for (int i = 0; i < 61; i++) {
				stringstream ss;
				ss << prefix << setw(4) << setfill('0') << i << ".jpg";
				string filename = ss.str();
				Mat img = imread(filename, IMREAD_GRAYSCALE);
				bookImgs.push_back(img);
			}
		}
		//==========================

		// VocabularyTree 객체 생성
		VocabularyTree VTree(10, 3, 32); // K, L, dimension 

		chrono::system_clock::time_point set_start = std::chrono::system_clock::now();
		// 데이터 설정
		try {
			VTree.setBookImgs(bookImgs, bookNames);
		}
		catch (exception & e) {
			cerr << e.what() << endl;
			return 1;
		}

		chrono::system_clock::time_point set_end = std::chrono::system_clock::now();
		chrono::duration<double> setting_time = set_end - set_start;
		cerr << "데이터 set 완료, 걸린 시간: " << setting_time.count() << endl;

		// fitting 시작
		cerr << "fitting 시작" << endl;
		chrono::system_clock::time_point fit_start = std::chrono::system_clock::now();
		VTree.fit();
		chrono::system_clock::time_point fit_end = std::chrono::system_clock::now();
		chrono::duration<double> fitting_time = fit_end - fit_start;
		cerr << "fitting 완료, 걸린 시간: " << fitting_time.count() << "초" << endl;
				
		// 모델 저장
		VTree.saveToFile();
		cout << "fitting된 모델 저장완료" << endl;

	}
	catch (exception & e) {
		cerr << e.what() << endl;
	}
	return 0;
}