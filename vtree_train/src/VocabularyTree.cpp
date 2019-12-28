#include "VocabularyTree.h"
#include "KMajorityData.h"
#include "UtilityFunctions.h"

#include <iostream>
#include <string>
#include <algorithm>
#include <vector>
#include <unordered_map>
#include <set>
#include <cmath>
#include <iterator>
#include <numeric> 
#include <bitset>
#include <sstream>
#include <fstream>
#include <memory>
#include "opencv2/opencv.hpp"

#include <chrono>

using namespace std;
using namespace cv;


// KMajority ==========================================================================
VocabularyTree::KMajority::KMajority(size_t _K, int _currLevel, VocabularyTree& _Master, size_t _DATA_DIMENSION)
	: K(_K), DATA_DIMENSION(_DATA_DIMENSION),
	currLevel(_currLevel),
	centers(K, std::vector<uchar>(DATA_DIMENSION, 0)),
	groupCnt(K, 0),
	parent(nullptr), children(K), master(_Master),
	dataIdxs(K),
	leafNodeID(-1),
	fittedNodeID(0)
{}

VocabularyTree::KMajority::~KMajority() = default;

void VocabularyTree::KMajority::setDatas(const std::set<int>& _dataIdxs, const std::vector<int>& _featureCntInFile) {
	totalDataIdxs = _dataIdxs;
	featureCntInFile = _featureCntInFile;

	// 그룹 초기화 (모두 그룹 0)
	dataIdxs.assign(K, set<int>());
	dataIdxs[0] = _dataIdxs;
	group.assign(master.datas.size(), 0);

}

void VocabularyTree::KMajority::initCenters() {
	// 처음 K 개 데이터로 중심점 초기화
	auto itr = totalDataIdxs.begin();
	for (int i = 0; i < K; i++) {
		centers[i] = master.datas[*itr].value; // 처음 K 개 데이터로 중심점 초기화
		itr++;
	}
}


void VocabularyTree::KMajority::fit() {
	// fit 정지조건
	constexpr uchar epsilon = 0;
	
	fittedNodeID = master.fittedNodesCnt;
	// 재귀 종료 조건
	if (totalDataIdxs.size() < K || currLevel >= master.L) {
		// leafnode에 추가
		master.addLeafNode(this);
		master.fittedNodesCnt++;
		master.showFittingProgress();
		return;
	}

	addChildren();

	// initializing
	initCenters();
	vector<vector<int>> fileNameCounterPerGroup(K, vector<int>(master.bookNames.size(), 0));
	fileNameCounterPerGroup[0] = featureCntInFile;

	// data 번호, k번호
	map<int, vector<size_t>> distances;
	for (int idx : totalDataIdxs) {
		distances[idx] = std::vector<size_t>(K, 0);
	}

	vector<vector<uchar>> old_centers(centers);

	while (true) {
		// 중심에서 각 데이터까지 거리 구하기
		for (int idx : totalDataIdxs) {
			for (int k = 0; k < K; k++) {
				distances[idx][k] = hammigDistance(centers[k], master.datas[idx].value);
			}
			// i번째 데이터에 가장 가까운 중심의 번호를 찾기
			int new_k = static_cast<int>(min_element(distances[idx].begin(), distances[idx].end()) - distances[idx].begin());
			int old_k = group[idx];

			// 가장 가까운 중심번호를 갱신
			if (old_k != new_k) {
				group[idx] = new_k;

				const string filename = master.datas[idx].filename;
				int fileIdx = master.bookNameVSIdx.at(filename);

				// 이전 그룹에서 제거
				dataIdxs[old_k].erase(idx);
				fileNameCounterPerGroup[old_k][fileIdx] -= 1;

				// 새 그룹에 추가
				dataIdxs[new_k].insert(idx);

				fileNameCounterPerGroup[new_k][fileIdx] += 1;

			}
		}

		// 그룹번호(k번호), 차원 번호
		vector<vector<unsigned long long>> center_bit_counters(K, vector<unsigned long long>(DATA_DIMENSION*8, 0));


		vector<int> _group_cnt(K, 0);
		for (int idx : totalDataIdxs) {
			_group_cnt[group[idx]]++;
		}

		for (int idx : totalDataIdxs) {
			for (int j = 0; j < DATA_DIMENSION; j++) {
				bitset<8> DataByte = master.datas[idx].value[j];
				for (int bit = 0; bit < 8; bit++) {
					if (DataByte[bit]) center_bit_counters[group[idx]][j * 8 + bit]++;
				}
			}
		}


		for (int k = 0; k < K; k++) {
			for (int i = 0; i < DATA_DIMENSION; i++) {
				bitset<8> temp(0);
				for (int bit = 0; bit < 8; bit++) {
					temp[bit] = center_bit_counters[k][i * 8 + bit] > _group_cnt[k] / 2 ? 1 : 0;
				}
				centers[k][i] = static_cast<uchar>(temp.to_ulong());
			}
		}

		

		bool Done = true;
		for (int k = 0; k < K; k++) {
			if (hammigDistance(old_centers[k], centers[k]) > epsilon) {
				Done = false;
				break;
			}
		}

		if (Done) {
			_group_cnt.swap(groupCnt);



			break;
		}
		else {
			old_centers = centers;
		}
	}

	master.fittedNodesCnt++;
	master.showFittingProgress();

	// 자식들 fit
	for (int k = 0; k < K; k++) {
		children[k]->setDatas(dataIdxs[k], fileNameCounterPerGroup[k]);
		// 재귀 호출
		children[k]->fit();
	}

}


void VocabularyTree::KMajority::addChildren() {
	for (int k = 0; k < K; k++) {
		auto& child = children[k];

		child = make_unique<KMajority>(K, currLevel + 1, master, DATA_DIMENSION);	
		child->parent = this;
	}
};


bool VocabularyTree::KMajority::isLeafNode() const {
	return leafNodeID != -1;
}

void VocabularyTree::KMajority::toBuff() const {
	//center info
	master.centersBuff << fittedNodeID << endl;
	for (int k = 0; k < K; k++) {
		for (int d = 0; d < DATA_DIMENSION; d++) {
			master.centersBuff << static_cast<int>(centers[k][d]);
			master.centersBuff << "|";
		}
		master.centersBuff << endl;
	}
	

	// tree structure
	master.treeSaveBuff << fittedNodeID << "(";
		
	if (isLeafNode()) {// 재귀 종료 조건
		master.treeSaveBuff << ")";
	}
	else {
		for (int k = 0; k < K; k++) {
			// 재귀 호출
			children[k]->toBuff();
			if(k != K-1) master.treeSaveBuff << ",";
		}
		master.treeSaveBuff << ")";
	}
}


//===============================================================

VocabularyTree::VocabularyTree(size_t _K, int _L, size_t _DATA_DIMENSION)
	: K(_K), L(_L), DATA_DIMENSION(_DATA_DIMENSION),
	  root_node(make_unique<KMajority>(K, 0, *this, DATA_DIMENSION)),
	  fittedNodesCnt(0),
	  maxTotalNodes(K!=1? static_cast<size_t>((pow(K, L+1)-1)/(K-1)) : L),
	  treeSaveBuff()
{}

VocabularyTree::~VocabularyTree() = default;

void  VocabularyTree::setBookImgs(const std::vector<cv::Mat>& bookImgs, const std::vector<std::string>& bookNames) {
	if (bookImgs.size() != bookNames.size()) throw("[setBookImgs] bookImgs.size() != bookNames.size()");

	this->bookNames = bookNames;
	vector<Data> datas;

	bookNameVSIdx.reserve(this->bookNames.size());
	for (int idx = 0; idx < this->bookNames.size(); idx++) {
		bookNameVSIdx.emplace(this->bookNames[idx], idx);
	}

	// 이미지 파일들 불러와서 특징점 추출후 vector<Data>에 추가
	for (int i = 0; i < this->bookNames.size(); i++) {
		if (bookImgs[i].empty()) {
			cerr << "Image is empty!: " << this->bookNames[i] << endl;
			throw runtime_error("[exception] Image load failed");
		}

		Mat src;

		// 리사이징
		resizeIfNecessary(bookImgs[i], src);

		// 특징점 추출(ORB)
		Ptr<Feature2D> feature = ORB::create();


		vector<KeyPoint> keypoints;

		Mat Matdesc;
		feature->detectAndCompute(src, Mat(), keypoints, Matdesc);

		vector<vector<uchar>> desc;
		MatTo2DVector(Matdesc, desc);

		if (desc.size() == 0) throw runtime_error(this->bookNames[i] + ", [setBookImgs] couldn't detect any key points");

		// datas에 추가
		for (const auto& v : desc) {
			datas.emplace_back(v, this->bookNames[i], int(v.size()));
		}
	}

	setDatas(datas);
}



void VocabularyTree::setDatas(const vector<Data>& datas) {
	this->datas = datas;

	size_t data_size = datas.size();
	set<int> dataIdxs;


	vector<int> featureCntInFile(bookNames.size(), 0);
	for (int i = 0; i < data_size; i++) {
		dataIdxs.insert(i);
		const string filename = datas[i].filename;
		int fileIdx = bookNameVSIdx.at(filename);

		featureCntInFile[fileIdx] += 1;
	}
	
	root_node->setDatas(dataIdxs, featureCntInFile);
}


void VocabularyTree::fit() {
	if (datas.size() == 0) {
		cerr << "[Error] There is no data. Please set the feature datas first" << endl;
		return;
	}

	root_node->fit();
	updateLeafNodeCnt();
	updateWeights();
	updateDatabaseImgVector();

	cout << "fitting complete" << endl;
	cout << "[INFO] Number of leaf nodes: " << leafNodes.size() << endl;
}


void VocabularyTree::addLeafNode(KMajority* node) {
	int id = static_cast<int>(leafNodes.size());
	leafNodes.push_back(node);
	node->leafNodeID = id;
}


void VocabularyTree::updateLeafNodeCnt() {
	const size_t leafNodesSize = leafNodes.size();
	leafNodeCnt.assign(leafNodesSize, 0);
	
	for (int i = 0; i < leafNodesSize; i++) {
		for (const auto& featureCnt : leafNodes[i]->featureCntInFile) {
			leafNodeCnt[i] += featureCnt != 0 ? 1 : 0;
		}
	}

}


void VocabularyTree::updateWeights() {

	const size_t leafNodesSize = leafNodes.size();
	weights.assign(leafNodesSize, 0);

	const int N = static_cast<int>(bookNames.size());

	for (int i = 0; i < leafNodesSize; i++) {
		weights[i] = log(N / leafNodeCnt[i]);
	}
}


void VocabularyTree::updateDatabaseImgVector() {

	const size_t leafNodesSize = leafNodes.size();
	databaseImgVector.assign(bookNames.size(), vector<double>(leafNodesSize, 0.0));

	for (int fileIdx = 0; fileIdx < bookNames.size(); fileIdx++) {

		for (int i = 0; i < leafNodesSize; i++) {

			const auto& featureCntInFile = leafNodes[i]->featureCntInFile;
			if (featureCntInFile.at(fileIdx) == 0) {
				continue;
			}

			databaseImgVector[fileIdx][i] = weights[i] * featureCntInFile.at(fileIdx);
		}

		// L1 Normalize the vector
		double L1 = L1Norm(databaseImgVector[fileIdx]);

		if (L1 != 0) std::transform(databaseImgVector[fileIdx].begin(), databaseImgVector[fileIdx].end(), databaseImgVector[fileIdx].begin(), [=](double value) { return value / L1; });
		
	}
	
}


void VocabularyTree::showFittingProgress() {
	size_t progress = fittedNodesCnt * 1000 / maxTotalNodes;
	if (progress % 100 == 0) {
		cout << "fitting progress: " << progress /10 << "%..." << endl;
	}
}


void VocabularyTree::saveToFile(const string& filename) {

	// reset buff
	treeSaveBuff.clear();
	treeSaveBuff.str("");
	centersBuff.clear();
	centersBuff.str("");


	// Save to buffers

	root_node->toBuff();

	centersBuff << "#CENTER_INFO_END";
	

	ofstream fout(filename);

	//line 1(K, L, DATA_DIMENSION, total number of leaf nodes )
	fout << K << " " << L << " " << DATA_DIMENSION << " " << leafNodes.size() << endl;

	//line 2(tree structure)
	fout << treeSaveBuff.rdbuf() << endl;

	//line 3(number of total books)
	fout << bookNames.size() << endl;

	//book names
	for (const auto& book : bookNames) {
		fout << book << endl;
	}

	//center info
	fout << centersBuff.rdbuf() << endl;

	//leaf node count
	for (const auto& cnt : leafNodeCnt) {
		fout << cnt << "|";
	}
	fout << endl;

	//weights
	for(const auto& w : weights){
		fout << w << "|";
	}
	fout << endl;


	//database Image vectors
	for (int idx = 0; idx < bookNames.size(); idx++) {
		for (int leafNode_i = 0; leafNode_i < leafNodes.size(); leafNode_i++) {
			fout << databaseImgVector[idx][leafNode_i];
			fout << "|";
		}
		fout << endl;
	}


}

