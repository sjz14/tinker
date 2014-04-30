# -*- coding: utf-8 -*-


修改方法：


编辑input.txt，写入可能出现的词语。
示例如下：
====================分割线====================
hello tinker
my name is
alex
bob
====================分割线====================
使用以下命令编译：
python input2dict.py


编辑gram.jsgf，写入语法。
示例如下：
====================分割线====================
#JSGF V1.0;
grammar furoc;
public <furocCmd> = <myname> | <hellotinker>;
<myname> = MY NAME IS <names>;
<names> = TOM | BOB | JACK;
<hellotinker> = HELLO <names>;
====================分割线====================
使用以下命令编译：
bash jsgf2fsg.sh


注意：编写input.txt和gram.jsgf时，单词请用大写。

