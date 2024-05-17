#include <iostream>

#include "config.h"

static Config config;

int main()
{
	config.load("config.ini");
}
