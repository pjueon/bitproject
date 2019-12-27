#pragma once
#ifndef VOCABULARY_TREE_H
#define VOCABULARY_TREE_H

#include <vector>
#include <set>
#include <unordered_map>
#include <string>
#include <sstream>
#include <memory>
#include "opencv2/opencv.hpp"

struct Data;

class VocabularyTree {
private:
	//================================================================
	struct KMajority {
		// methods
		explicit KMajority(size_t _K, int _currLevel, VocabularyTree& _master, size_t _DATA_DIMENSION);
		~KMajority();
		void setDatas(const std::set<int>&, const std::vector<int>&);
		void initCenters();
		void fit();
		void addChildren();
		bool isLeafNode() const;
		void toBuff() const;

		// fields
		const size_t DATA_DIMENSION;
		const size_t K;
		std::set<int> totalDataIdxs;						// ���� ��忡�� �з��ؾ��ϴ� �� �����͵��� idx 
		std::vector<std::vector<unsigned char>> centers;	// Group��ȣ(k ��ȣ), ���� ��ȣ �� ���� �߽���ǥ
		std::vector<int> group;							    // idx: ������ idx, value : k
		std::vector<int> groupCnt;							// �׷캰(k) ũ��
		const int currLevel;
		std::vector<std::set<int>> dataIdxs;				// �׷캰(k) ������ �ε��� 
		std::vector<int> featureCntInFile;			        // ���� idx : value ����
		int leafNodeID;                                     // leaf node�� �ƴ� ��� -1
		size_t fittedNodeID;

		KMajority* parent;
		std::vector<std::unique_ptr<KMajority>>children;

		VocabularyTree& master;
	};
	//================================================================
public:
	VocabularyTree(size_t _K, int _L, size_t _DATA_DIMENSION);
	~VocabularyTree();

	void setBookImgs(const std::vector<cv::Mat>& bookImgs, const std::vector<std::string>& bookNames);

	void fit();
	void saveToFile(const std::string& filename = "imgdb.vtree");



private:
	// methods
	void setDatas(const std::vector<Data>& datas);
	void addLeafNode(KMajority*);
	void updateLeafNodeCnt();
	void updateWeights();
	void updateDatabaseImgVector();
	void showFittingProgress();

	// fields
	const size_t DATA_DIMENSION;
	const size_t K;
	const size_t L;
	std::vector<Data> datas;
	std::vector<std::string> bookNames;
	std::unordered_map<std::string, int> bookNameVSIdx;

	std::unique_ptr<KMajority> root_node;
	std::vector<KMajority*> leafNodes;
	std::vector<double> weights;									// The idx is leafNodeID
	std::vector<std::vector<double>> databaseImgVector;	            // databaseImgVector[file index][leafNodeID]

	std::vector<int> leafNodeCnt;									// The idx is leafNodeID
	size_t fittedNodesCnt;
	const size_t maxTotalNodes;
	std::stringstream treeSaveBuff;
	std::stringstream centersBuff;
	std::stringstream leafNodeIDBuff;

};



#endif