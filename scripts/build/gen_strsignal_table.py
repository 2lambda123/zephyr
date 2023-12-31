#!/usr/bin/env python3
#
# Copyright (c) 2023 Meta
#
# SPDX-License-Identifier: Apache-2.0

import argparse
import os
import re


def front_matter():
    return f'''
/*
 * This file is generated by {__file__}
 */

#include <zephyr/posix/signal.h>
'''


def gen_strsignal_table(input, output):
    with open(input, 'r') as inf:

        highest_signo = 0
        symbols = []
        msgs = {}

        for line in inf.readlines():
            # Select items of the form below (note: SIGNO is numeric)
            # #define SYMBOL SIGNO /**< MSG */
            pat = r'^#define[\s]+(SIG[A-Z_]*)[\s]+([1-9][0-9]*)[\s]+/\*\*<[\s]+(.*)[\s]+\*/[\s]*$'
            match = re.match(pat, line)

            if not match:
                continue

            symbol = match[1]
            signo = int(match[2])
            msg = match[3]

            symbols.append(symbol)
            msgs[symbol] = msg

            highest_signo = max(int(signo), highest_signo)

        try:
            os.makedirs(os.path.dirname(output))
        except BaseException:
            # directory already present
            pass

        with open(output, 'w') as outf:

            print(front_matter(), file=outf)

            # Generate string table
            print(
                f'static const char *const strsignal_list[{highest_signo + 1}] = {{', file=outf)
            for symbol in symbols:
                print(f'\t[{symbol}] = "{msgs[symbol]}",', file=outf)

            print('};', file=outf)


def parse_args():
    parser = argparse.ArgumentParser(allow_abbrev=False)
    parser.add_argument(
        '-i',
        '--input',
        dest='input',
        required=True,
        help='input file (e.g. include/zephyr/posix/signal.h)')
    parser.add_argument(
        '-o',
        '--output',
        dest='output',
        required=True,
        help='output file (e.g. build/zephyr/misc/generated/lib/posix/strsignal_table.h)')

    args = parser.parse_args()

    return args


def main():
    args = parse_args()
    gen_strsignal_table(args.input, args.output)


if __name__ == '__main__':
    main()
