// project:     followme_select
// file:        main.cpp
// created by bss at 2013-12-22
// Last modified: 2014-07-11, 10:47:33
// description: 

#include <stdio.h>
#include "followme/followme.h"

int main(int argc, char** argv)
{
    FollowmeDN* follow = new FollowmeDN();
    bool flag = follow->Init(argc, argv);

    delete follow;
    printf("Bye!\n");
    return 0;
}

