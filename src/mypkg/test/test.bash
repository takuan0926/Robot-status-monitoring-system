#!/bin/bash
# SPDX-FileCopyrightText: 2024 suzuki takuma <s23c1076vc@s.chibakoudai.jp>
# SPDX-License-Identifier: BSD-3-Clause

ng () {
    echo "${1}行目が違うよ"
    res=1
}

res=0

#正常なとき
out=$(echo "10000 10" | python3 ./tiac)
if [[ "$out" != *"税込み: 11000.00円"* ]]; then
    ng "$LINENO"
fi

#不正な入力なとき
out=$(echo "not_a_number 10" | python3 ./tiac 2>&1)
if [[ "$out" != *"エラー: 数値を入力してください。"* ]]; then
    ng "$LINENO"
fi

#結果表示
if [ "${res}" = 0 ]; then
    echo OK
else
    exit 1
fi
