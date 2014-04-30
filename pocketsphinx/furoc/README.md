修改方法
=====
<br />
<br />
dict
-----
编辑input.txt，写入可能出现的词语。<br />
示例如下：<br />
====================分割线====================<br />
hello tinker<br />
my name is<br />
alex<br />
bob<br />
====================分割线====================<br />
使用以下命令编译：<br />
python input2dict.py<br />
<br />
<br />
fsg
-----
编辑gram.jsgf，写入语法。<br />
示例如下：<br />
====================分割线====================<br />
&#35;JSGF V1.0;<br />
grammar furoc;<br />
public <furocCmd> = <myname> | <hellotinker>;<br />
<myname> = MY NAME IS <names>;<br />
<names> = TOM | BOB | JACK;<br />
<hellotinker> = HELLO <names>;<br />
====================分割线====================<br />
使用以下命令编译：<br />
bash jsgf2fsg.sh<br />
<br />
<br />
注意：编写input.txt和gram.jsgf时，单词请用大写。<br />

