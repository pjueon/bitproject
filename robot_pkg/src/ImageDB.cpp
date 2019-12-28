#include "ImageDB.h"
#include "opencv2/opencv.hpp"
#include "UtilityFunctions.h"

#include <cmath>
#include <numeric> 
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <algorithm>
#include <vector>
#include <set>
//#include <bitset>
#include <chrono>

using namespace std;
using namespace cv;

//============================Node============================

ImageDB::Node::Node(size_t _K, int _currLevel, ImageDB& _master, size_t _DATA_DIMENSION)
	:K(_K), DATA_DIMENSION(_DATA_DIMENSION), currLevel(_currLevel), centers(K, std::vector<uchar>(DATA_DIMENSION, 0)),
	parent(nullptr), children(K), master(_master), leafNodeID(-1), nodeID(0)
{}

ImageDB::Node::~Node() = default;

int ImageDB::Node::findK(const vector<uchar>& input) const {
	int ret = 0;
	size_t minValue = 300;

	for (int k = 0; k < K; k++) {
		auto value = hammigDistance(centers[k], input);
		if (value < minValue) {
			ret = k;
			minValue = value;
		}
	}

	return ret;
}

bool ImageDB::Node::isLeafNode() const {
	return leafNodeID != -1;
}

void ImageDB::Node::buildFromFile() {
	char data;
	vector<unique_ptr<Node>> newChildren(K);

	stringstream ss;
	int k = 0;
	for (auto& child : newChildren) {
		while (true) {
			master.treeImportBuff >> data;
			auto pos = master.treeImportBuff.tellg();

			if (data == ')') {

				break;
			}
			else if (data == ',') {
				continue;
			}
			else if (data == '(') {
				char nextData;
				master.treeImportBuff >> nextData;
				// reset the position
				master.treeImportBuff.seekg(pos);

				child = make_unique<Node>(K, currLevel + 1, master, DATA_DIMENSION);

				ss >> child->nodeID;
				child->centers = master.centerInfo[static_cast<int>(child->nodeID)];



				ss.clear();
				ss.str("");

				if (nextData == ')') {
					child->leafNodeID = static_cast<int>(master.leafNodes.size());
					master.leafNodes.emplace_back(this);
				}
				else {
					child->leafNodeID = -1;
					child->buildFromFile();
				}
			}
			else {// number
				ss << data;
			}
		}
	}

	children = move(newChildren);
}



//============================Node============================

//==========================ImageDB===========================
ImageDB::ImageDB()
	: K(0), L(0), DATA_DIMENSION(0),
	root_node()
{}

ImageDB::~ImageDB() = default;


int ImageDB::findNearistLeafNode(const std::unique_ptr<Node>& node, const std::vector<unsigned char>& input) const {
		if (node->isLeafNode()) {
			return node->leafNodeID;
		}
		else {
			auto k = node->findK(input);
			return findNearistLeafNode(node->children.at(k), input);
		}
}

string ImageDB::findImg(const vector<vector<uchar>>& input) const {
	vector<int> travelResult(leafNodes.size(), 0);

	for (const auto& v : input) {
		int node_num = findNearistLeafNode(root_node, v);
		travelResult[node_num]++;
	}

	// Make Query vector
	vector<double> query(leafNodeCnt.size(), 0.0);

	for (int node_num = 0; node_num < leafNodes.size(); node_num++) {
		query[node_num] = weights[node_num] * travelResult[node_num];
	}

	// L1 Normalize the vector
	double L1 = L1Norm(query);
	if (L1 != 0) std::transform(query.begin(), query.end(), query.begin(), [=](double value) { return value / L1; });

	// Scoring
	vector <double> Scores(files.size(), 0.0);

	for (int node_num = 0; node_num < leafNodes.size(); node_num++) {
		for (int fileIdx = 0; fileIdx < files.size(); fileIdx++) {

			auto qi = query[node_num];
			auto di = databaseImgVector.at(fileIdx)[node_num];
			if (qi != 0 && di != 0) {
				Scores[fileIdx] += abs(qi - di) - abs(qi) - abs(di);
			}
		}
	}


	auto winner = std::min_element(Scores.begin(), Scores.end()) - Scores.begin();

	return files[winner];
}

std::string ImageDB::search(const cv::Mat& img) const {
	if (img.empty()) {
		cerr << "Image is empty: " << endl;
		return "error";
	}

	Mat src;

	// 리사이징
	resizeIfNecessary(img, src);

	chrono::system_clock::time_point surf_start = std::chrono::system_clock::now();

	// 특징점 추출(ORB)
	Ptr<Feature2D> feature = ORB::create();
	vector<KeyPoint> keypoints;
	Mat Matdesc;
	feature->detectAndCompute(src, Mat(), keypoints, Matdesc);

	chrono::system_clock::time_point surf_end = std::chrono::system_clock::now();
	chrono::duration<double> feature_extracting_time = surf_end - surf_start;

	// DEBUG 시간측정
	cerr << "[search] feature_extracting_time = " << feature_extracting_time.count() << endl;

	if (Matdesc.empty()) {
		throw runtime_error("There is no Keypoint in the image.");
	}


	// 추출한 특징점들을 Mat 형식에서 vector<vector<uchar>>으로 변환
	vector<vector<uchar>> desc;
	MatTo2DVector(Matdesc, desc);

	return findImg(desc);


}


string ImageDB::search(const std::string& filename) const {
	// 파일 열기
	Mat src = imread(filename, IMREAD_GRAYSCALE);

	if (src.empty()) {
		cerr << "Image load failed: " << filename << endl;
		return "error";
	}

	return search(src);
}

void ImageDB::load(const std::string& filename) {

	ifstream fin(filename);
	if (!fin.is_open()) {
		cerr << "couldn't open" << filename << endl;
		return;
	}

	try {
		vector<string> lines;
		string line;

		while (getline(fin, line)) {
			lines.push_back(line);
		}

		size_t new_K, new_L, new_DATA_DIMENSION, total_num_of_leafNodes;

		{
			stringstream ss(lines[0]);
			ss >> new_K;
			ss >> new_L;
			ss >> new_DATA_DIMENSION;
			ss >> total_num_of_leafNodes;
		}

		K = new_K;
		L = new_L;
		DATA_DIMENSION = new_DATA_DIMENSION;
		cout << "K: " << K << ", L: " << L << ", DATA_DIMENSION: " << DATA_DIMENSION << endl;
		cout << "total_num_of_leafNodes: " << total_num_of_leafNodes << endl;


		//tree structure		
		treeImportBuff.str(lines[1]);


		root_node = make_unique<Node>(new_K, 0, *this, new_DATA_DIMENSION);
		treeImportBuff >> root_node->nodeID;

		char tmp;
		treeImportBuff >> tmp; // '('

		cout << "[DEBUG]트리 구조 읽어오기 성공 " << endl;

		//files
		int num_of_book_Imgs = stoi(lines[2]);
		cout << "number of book imgs = " << num_of_book_Imgs << endl;
		int line_num = 3;

		for (; line_num < 3 + num_of_book_Imgs; line_num++) {
			files.push_back(lines[line_num]);
		}


		cout << "[DEBUG]파일명 읽어오기 성공" << endl;

		int l1 = line_num;

		while (lines[line_num] != "#CENTER_INFO_END") {
			int id = stoi(lines[line_num++]);

			vector<vector<unsigned char>>newCenters(new_K, vector<unsigned char>(new_DATA_DIMENSION, 0));
			
			for (int k = 0; k < new_K; k++) {
				stringstream ss(lines[line_num++]);
				string tmp;
				for (int d = 0; d < new_DATA_DIMENSION; d++) {
					getline(ss, tmp, '|');
					newCenters[k][d] = static_cast<unsigned char>(stoi(tmp));
				}
			}
			centerInfo[id] = newCenters;
		}

		root_node->centers = centerInfo[static_cast<int>(root_node->nodeID)];

		cout << "[DEBUG]center 정보 읽어오기 성공" << endl;

		//leaf node count
		{
			leafNodeCnt.assign(total_num_of_leafNodes, 0);

			stringstream ss(lines[++line_num]);
			string tmp;

			for (int leafNode_i = 0; leafNode_i < total_num_of_leafNodes; leafNode_i++) {
				getline(ss, tmp, '|');
				//cout << "leafNodeCnt[" << leafNode_i << "] = " << stoi(tmp) << endl;
				leafNodeCnt[leafNode_i] = stoi(tmp);
			}

		}
		cout << "[DEBUG]leaf node count 정보 읽어오기 성공" << endl;


		//weights
		{
			
			weights.assign(total_num_of_leafNodes, 0.0);

			stringstream ss(lines[++line_num]);
			string tmp;

			for (int leafNode_i = 0; leafNode_i < total_num_of_leafNodes; leafNode_i++) {
				getline(ss, tmp, '|');
				weights[leafNode_i] = stod(tmp);
			}

		}
		cout << "[DEBUG]weights 정보 읽어오기 성공" << endl;
		line_num++;

		//databaseImgVector
		{
			databaseImgVector.assign(files.size(), vector<double>(total_num_of_leafNodes, 0.0));
			for (int i = 0; i < files.size(); i++) {
				stringstream ss(lines[line_num++]);
				string tmp;

				for (int leafNode_i = 0; leafNode_i < total_num_of_leafNodes; leafNode_i++) {
					getline(ss, tmp, '|');
					databaseImgVector[i][leafNode_i] = stod(tmp);
				}

			}
		}
		cout << "[DEBUG]databaseImgVector 정보 읽어오기 성공" << endl;
		

		// build
		root_node->buildFromFile();
	}
	catch (exception & e) {
		cerr << "[Exception from load()]" << e.what() << endl;
	}
}

//==========================ImageDB===========================