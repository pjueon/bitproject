#include "Logger.h"
#include "MainMachine.h"
#include <iostream>
using namespace std;

Logger::Logger(MainMachine* const mainMachine, const string moduleName)
	:mainMachine(mainMachine), moduleName(moduleName)
{}

Logger::~Logger() {}