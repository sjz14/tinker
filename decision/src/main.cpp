// project:     decide
// file:        main.cpp
// created by gjq at 2014-1-18
// description: 

#include <stdio.h>
#include "decide.h"

int main(int argc, char** argv)
{
    Decide* decide = new Decide();
    //ros::init(argc, argv, "decision");
    
    bool flag = decide->Init(argc, argv);

    if (flag)
    {
        decide->Run();
    }

    delete decide;
    printf("Bye!\n");
    return 0;
}

