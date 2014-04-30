// project:     followme
// file:        main.cpp
// created by bss at 2013-12-22
// last edit: by bss at 2013-12-22
// description: 

#include <stdio.h>
#include "followme.h"

int main(int argc, char** argv)
{
    FollowmeDN* follow = new FollowmeDN();
    bool flag = follow->Init(argc, argv);

    if (flag)
    {
        follow->Run();
    }

    delete follow;
    printf("Bye!\n");
    return 0;
}

