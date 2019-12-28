#pragma once
#ifndef IMAGE_DB_H
#define IMAGE_DB_H

#include <vector>
#include <string>
#include <sstream>
#include <map>
#include <opencv2/opencv.hpp>
#include <memory>

class ImageDB {
private:
	//==========================================================================
	struct Node {
		Node(size_t _K, int _currLevel, ImageDB& _master, size_t _DATA_DIMENSION);
		~Node();
		int findK(const std::vector<unsigned char>& input) const;
		void buildFromFile();
		bool isLeafNode() const;


		const size_t DATA_DIMENSION;
		const size_t K;
		const int currLevel;

		size_t nodeID;
		int leafNodeID;											    // leaf node가 아닌 경우 -1
		std::vector<std::vector<unsigned char>> centers;			// Group번호(k 번호), 차원 번호 에 대한 중심좌표

		Node* parent;
		std::vector<std::unique_ptr<Node>> children;
		ImageDB& master;

	};
	//==========================================================================

public:
	ImageDB();
	~ImageDB();
	std::string search(const std::string& filename) const;
	std::string search(const cv::Mat& img) const;
	void load(const std::string& filename = "imgdb.vtree");

private:
	int findNearistLeafNode(const std::unique_ptr<Node>& node, const std::vector<unsigned char>& input) const; // returns leaf node ID
	std::string findImg(const std::vector<std::vector<unsigned char>>&) const;

	// fields
	size_t DATA_DIMENSION;
	size_t K;
	size_t L;

	std::map<int, std::vector<std::vector<unsigned char>>> centerInfo;

	std::vector<std::string> files;
	std::unique_ptr<Node> root_node;

	std::vector<Node*> leafNodes;
	std::vector<double> weights;
	std::vector<std::vector<double>> databaseImgVector;
	std::vector<int> leafNodeCnt;									// The idx is leafNodeID
	std::stringstream treeImportBuff;
};

#endif