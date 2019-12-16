#include "AStar.h"
#include "ros/ros.h"

#include <vector>
#include <utility>
#include <tuple>
#include <algorithm>

#include <iostream>


constexpr bool CLEAR = true;
constexpr bool BLOCKED = false;

using namespace std;

//====================================
AStar::Node_t::Node_t():current_x(0), current_y(0), parent_x(0), parent_y(0), f(0), g(0), h(0){}

//-----------
AStar::Node_t::Node_t(int _current_x, int _current_y)
:current_x(_current_x), current_y(_current_y), parent_x(0), parent_y(0), f(0), g(0), h(0) {}

//-----------
bool AStar::Node_t::operator==(const Node_t& other) const{
	return current_x == other.current_x && current_y == other.current_y;
}

//-----------
bool AStar::Node_t::operator!=(const Node_t& other) const{
	return !(*this == other);
}

//-----------
bool AStar::closeListComp::operator()(const Node_t &a, const Node_t &b) const {
	if(a.current_x != b.current_x)
		return a.current_x < b.current_x;
	else 
		return a.current_y < b.current_y;
}

//-----------
bool AStar::openListComp::operator()(const Node_t &a, const Node_t &b) const {
	if(a.f != b.f)
		return a.f < b.f;
	else
		return closeComp(a, b);
}
//====================================


//-----------
void AStar::setSimpleMap(const std::vector<std::vector<bool>>& map){
	simpleMap = map;
	map_height = static_cast<int>(map.size());
	map_width = map_height > 0? static_cast<int>(map[0].size()) : 0;
}

//-----------
void AStar::setStartPoint(int x, int y){
	std::tie(start_x, start_y) = getNearistClearPoint(x, y);

	cout << "start: "<< start_x << ", " << start_y << endl;
}

//-----------
void AStar::setGoal(int x, int y){
	std::tie(goal_x, goal_y) = getNearistClearPoint(x, y);

	cout << "goal: "<< goal_x << ", " << goal_y << endl;
}

bool AStar::ok() const{
	return ros::ok();
	//return true;
}


//-----------
std::pair<int, int> AStar::getNearistClearPoint(int x, int y) const {
	if(!isValidXY(x, y)){
		std::cout<<"invalid x y!!!"<<std::endl;
		std::cout<<"x:" << x << ", y: " << y <<std::endl;
		throw std::runtime_error("invalid input");
	}	

    //BFS
	std::queue<std::pair<int, int>> q;
	std::vector<std::vector<bool>> visited(map_height, std::vector<bool>(map_width, false));

	q.push({x, y});
    
	while(!q.empty() && ok()){
		auto now = q.front();
	    q.pop();

		if(visited[now.second][now.first]) continue;

		if(simpleMap[now.second][now.first] == CLEAR) return now;

		visited[now.second][now.first] = true;

		int tmp_x = now.first;
		int tmp_y = now.second;

		std::vector<std::pair<int, int>> NodeList = {
														{tmp_x+1, tmp_y}, {tmp_x-1, tmp_y}, {tmp_x, tmp_y+1}, {tmp_x, tmp_y-1},
														{tmp_x+1, tmp_y+1}, {tmp_x+1, tmp_y-1}, {tmp_x-1, tmp_y+1}, {tmp_x-1, tmp_y+1}
													};

		for(const auto& p :NodeList)
			if(isValidXY(p.first, p.second) && !visited[p.second][p.first])	q.push(p);

	}

    throw std::runtime_error("Can't find clear point");
}




//-----------
bool AStar::isSameXY(int x1, int y1, int x2, int y2) const {
    return x1 == x2 && y1 == y2;
}


//-----------
int AStar::calculateH(int x, int y){
    return abs(goal_x - x) + abs(goal_y - y);
}

//-----------
bool AStar::isInCloseList(const Node_t &now) const{
    auto itr = closeList.find(now);
    return itr != closeList.end();
}

//-----------
bool AStar::pushToOpenList(int x, int y, int gScore){
	if(!isValidXY(x, y) || simpleMap[y][x] == BLOCKED) return false;
	

	Node_t newNode(x, y);
	newNode.g = gScore;
	newNode.h = calculateH(x, y);
	newNode.f = newNode.g + newNode.h;
	newNode.parent_x = current_x;
	newNode.parent_y = current_y;
	
	if(!isInCloseList(newNode)){
		auto itr = find_if(openList.begin(), openList.end(), [newNode](auto node) { return node == newNode; });		
		if(itr != openList.end()){ // is in openList?
			if(gScore < itr->g) //is better than old one?
				openList.erase(itr);			
			else
				return true;
		}

		openList.insert(newNode);
		return true;
	}
	return false;
}


//-----------
bool AStar::startSearch(){
	openList.clear();
	closeList.clear();

	current_x = start_x;
	current_y = start_y;
	pushToOpenList(start_x, start_y, 0);

	while (true) {
		auto currentNode_itr = openList.begin();

		if (currentNode_itr == openList.end()) {
			cerr << "fail to find the route" << endl;
			return false;
		}

		auto currentNode = *currentNode_itr;

		current_x = currentNode.current_x;
		current_y = currentNode.current_y;
		
		auto current_g = currentNode.g;

		openList.erase(currentNode_itr);
		closeList.insert(currentNode);

		if (isSameXY(current_x, current_y, goal_x, goal_y)) {
			std::cout << "Found the route!!" << std::endl << std::endl;
			return true;
		}

		bool rightClear = pushToOpenList(current_x + 1, current_y, current_g + 10);
		bool leftClear = pushToOpenList(current_x - 1, current_y, current_g + 10);
		bool upClear = pushToOpenList(current_x, current_y + 1, current_g + 10);
		bool downClear = pushToOpenList(current_x, current_y - 1, current_g + 10);

		if (rightClear && upClear) pushToOpenList(current_x + 1, current_y + 1, current_g + 14);
		if (rightClear && downClear) pushToOpenList(current_x + 1, current_y - 1, current_g + 14);
		if (leftClear && upClear) pushToOpenList(current_x - 1, current_y + 1, current_g + 14);
		if (leftClear && downClear) pushToOpenList(current_x - 1, current_y - 1, current_g + 14);
	}
}

//-----------
std::vector<std::pair<int, int>> AStar::findRoute(){
	bool success = startSearch();
	if(success) return getRoute();
	
	std::cout << "fail to find route" << std::endl;
	return {};
}

//----------- 

std::vector<std::pair<int, int>> AStar::getRoute(){
	cout << "AStar::getRoute() start!" << endl;

	std::vector<std::pair<int, int>> route;

	route.emplace_back(goal_x, goal_y);

	int dx = 100, dy = 100;
	int prev_x = goal_x, prev_y = goal_y;
	Node_t node(goal_x, goal_y);

	while (ok()) {
		auto itr = closeList.find(node);

		if (itr == closeList.end()) break;
		

		node.current_x = itr->parent_x;
		node.current_y = itr->parent_y;


		if (dx == node.current_x - prev_x && dy == node.current_y - prev_y) route.pop_back();

		route.emplace_back(node.current_x, node.current_y);

		dx = node.current_x - prev_x;
		dy = node.current_y - prev_y;

		prev_x = node.current_x;
		prev_y = node.current_y;

		if (isSameXY(node.current_x, node.current_y, start_x, start_y)) break;
	}

	std::reverse(route.begin(), route.end());
	cout << "AStar::getRoute() end!" << endl;	
	return route;
}


//-----------
bool AStar::isValidXY(int x, int y) const{
    return 0 <= x && x < map_width && 0 <= y && y < map_height;

}






