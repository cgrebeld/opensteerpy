/*
 *  OpenSteerLib.cp
 *  OpenSteerLib
 *
 *  Created by Grebeldinger on 13-11-13.
 *
 *
 */

#include <iostream>
#include "OpenSteerLib.h"
#include "OpenSteerLibPriv.h"

void OpenSteerLib::HelloWorld(const char * s)
{
	 OpenSteerLibPriv *theObj = new OpenSteerLibPriv;
	 theObj->HelloWorldPriv(s);
	 delete theObj;
};

void OpenSteerLibPriv::HelloWorldPriv(const char * s) 
{
	std::cout << s << std::endl;
};

