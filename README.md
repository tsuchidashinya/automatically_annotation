# アノテーションの自動化
自分たちで運営してきたシステムに深層学習を適用したいというときにぶつかる問題がアノテーションをどうするかである。アノテーションとは教師あり学習のときの学習させるデータセットに対して正解ラベルを付与する作業のことを一般的にはいう。私はこれまでROSのシミュレータ環境を用いてアノテーションを自動化するプログラムを開発してきた。以下にこれまで実装してきたアノテーションの自動化プログラムを紹介する

## 画像物体検出のアノテーション
<p align="center">
<img src="https://github.com/tsuchidashinya/automatically_annotation/blob/main/video/output.gif"></img></p>

### 原理
画像物体検出は深層学習の代表的なタスクの一つで画像中に求める物体があったときのその領域を矩形(バウンディングボックス)で囲む処理を行う。したがって、正解データでは、バウンディングボックスの２頂点の座標が必要とされる。

シミュレータを用いると物体の3次元座標の正しい値を取得することができる。その3次元データから下の図のような幾何学変換を用いて、画像中の座標データに変換することでバウンディングボックスを囲んでいる。
<p align="center">
<img src="https://github.com/tsuchidashinya/automatically_annotation/blob/main/video/image_1.png" width="600px" height="300px" ></img></p>

### 実装コマンド
ROSがインストールされている環境であれば、以下のコマンドで実行することができる。
```
roslaunch yolo_annotation kai_yolo_annotation_all_HV8.launch
```

## 点群のセマンティックセグメンテーションを行うアノテーション
<p align="center">
<img src="https://github.com/tsuchidashinya/automatically_annotation/blob/main/video/output_1.gif"></img></p>