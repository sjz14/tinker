修改方法
=====
<br />
<br />
dict
-----
编辑input.txt，写入可能出现的词语。<br />
示例如下：<br />

    hello tinker
    my name is
    alex
    bob

使用以下命令编译：

    python input2dict.py

<br />
<br />
fsg
-----
编辑gram.jsgf，写入语法。<br />
示例如下：

    #JSGF V1.0;
    grammar furoc;
    public <furocCmd> = <myname> | <hellotinker>;
    <myname> = MY NAME IS <names>;
    <names> = TOM | BOB | JACK;
    <hellotinker> = HELLO <names>;

使用以下命令编译：

    bash jsgf2fsg.sh

<br />
<br />
注意：编写input.txt和gram.jsgf时，单词请用大写。<br />

