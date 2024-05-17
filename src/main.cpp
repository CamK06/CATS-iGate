#include <iostream>
#include <flog.h>

#include "config.h"

static Config config;

int main()
{
	flog::info("Starting iGate...");
	if(!config.load("config.ini")) {
		flog::error("Failed to load config");
		flog::error("Quitting...");
		return -1;
	}
}
