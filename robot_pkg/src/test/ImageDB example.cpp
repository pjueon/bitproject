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

		// ImageDB 객체 생성
		ImageDB db;

		auto load_start = std::chrono::system_clock::now();
		// 파일 읽어오기
		db.load();
		auto load_end = std::chrono::system_clock::now();
		chrono::duration<double> loading_time = load_end - load_start;
		cout << "DB 로딩 완료, 시간: " << loading_time.count() << endl;


		set<string> queryFileNames;
		//========== 임시 ==========
		{
			string prefix = "img/test/test";
			for (int i = 0; i < 7; i++) {
				stringstream ss;
				ss << prefix << setw(4) << setfill('0') << i << ".jpg";
				queryFileNames.insert(ss.str());
			}
		}
		//==========================
		
		for (const auto& queryFilename : queryFileNames) {
			cout << "=========================" << endl;
			cout << "Query Filename : " << queryFilename << endl;
			chrono::system_clock::time_point query_start = std::chrono::system_clock::now();

			string result = db.search(imread(queryFilename, IMREAD_GRAYSCALE));
			//string result = db.search(queryFilename);

			chrono::system_clock::time_point query_end = std::chrono::system_clock::now();
			chrono::duration<double> query_time = query_end - query_start;

			cout << "result: " << result << ", 찾는데 걸린 시간: " << query_time.count() << "초" << endl << endl;

			Mat queryImg = imread(queryFilename);
			imshow("query image", queryImg);

			waitKey();
			destroyAllWindows();
		}

	}
	catch (exception & e) {
		cerr << e.what() << endl;
	}
	return 0;
}