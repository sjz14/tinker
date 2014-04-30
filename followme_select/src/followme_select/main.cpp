// project:     followme_select
// file:        main.cpp
// created by bss at 2013-12-22
// Last modified: 2014-04-20, 01:05:08
// description: 

#include <stdio.h>
#include "followme.h"

int main(int argc, char** argv)
{
    FollowmeDN* follow = new FollowmeDN();
    bool flag = follow->Init(argc, argv);

    delete follow;
    printf("Bye!\n");
    return 0;
}

