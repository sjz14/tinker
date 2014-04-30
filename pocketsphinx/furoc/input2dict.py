#!/usr/bin/env python
# -*- coding: utf-8 -*-
# file: input2dict.py
# created by bss at 2014-04-30
# Last modified: 2014-04-30, 14:05:33
# 把输入转化为字典
#

if __name__ == '__main__':
    fp = open('cmudict_SPHINX_40', 'r')
    cmudict = {}
    for line in fp.readlines():
        col = line.strip().upper().split('\t', 1)
        cmudict[col[0]] = col[1]
    fp.close()

    fp = open('input.txt', 'r')
    words = []
    for line in fp.readlines():
        col = line.strip().upper().split()
        for word in col:
            words.append(word)
    fp.close()

    words = list(set(words))

    fp = open('words.dic', 'w')
    for word in words:
        try:
            line = word + '\t' + cmudict[word] + '\n'
            fp.write(line)
            for i in range(2, 5):
                key = word + '(' + str(i) + ')'
                line = key + '\t' + cmudict[key] + '\n'
                fp.write(line)
        except:
            pass
    fp.close()

