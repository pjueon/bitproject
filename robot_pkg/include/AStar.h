#include<cmath>
#include<vector>
#include<set>
#include<utility>
#include<string>
#include <queue>


class AStar{
private:
//====================================
struct Node_t{
	int current_x, current_y;
	int parent_x, parent_y;
	int f;
	int g;
	int h;
	Node_t();
	Node_t(int _current_x, int _current_y);

	bool operator==(const Node_t& other) const;
	bool operator!=(const Node_t& other) const;
};


struct closeListComp{
    bool operator()(const Node_t &a, const Node_t &b) const;
};


struct openListComp{
    bool operator()(const Node_t &a, const Node_t &b) const;
private:
    closeListComp closeComp;
};

//====================================


private:
	int start_x;
	int start_y;

	int goal_x;
	int goal_y;

	int current_x;
	int current_y;

	std::vector<std::vector<bool>> simpleMap;

	int map_height;
	int map_width;

	std::set<Node_t,openListComp> openList;
	std::set<Node_t,closeListComp> closeList;

	std::pair<int, int> getNearistClearPoint(int x, int y) const;
	bool isSameXY(int x1, int y1, int x2, int y2) const;

	int calculateH(int x, int y);
	bool isInCloseList(const Node_t& now) const;
	bool startSearch();
	std::vector<std::pair<int, int>> getRoute();	
	bool pushToOpenList(int x, int y, int gScore);
	bool isValidXY(int x, int y) const;
	bool ok() const;


public:
	void setSimpleMap(const std::vector<std::vector<bool>>& map);
	void setStartPoint(int x, int y);
	void setGoal(int x, int y);

	std::vector<std::pair<int, int>> findRoute();

};


