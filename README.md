# MageSimulator
MAGE Simulatorは、Gコードと対象とする3Dプリンタに関する情報を読み込み、動作をシミュレートするソフトウェアです。
MAGE Simulatorでは、動作をシミュレートする際に、造形物と3Dプリンタ自体の衝突検出を行うことができます。
一般的な3軸の3Dプリンタは、水平面を鉛直方向に積層していく動作をするだけなため、干渉することは起こりえません。
しかしながら、多軸の3Dプリンタでは動作が複雑になり、既に造形された部分にヘッドが動いたり、造形ベッドとヘッドが干渉するなどの機械的な干渉が起こりえます。
そのため、多軸の3Dプリンタでは衝突検出が必要になります。
なお、MAGE Simulatorは[MAGE Interface](https://github.com/gear2nd-droid/MageInterface)から起動することができるようになっています。

## MAGEとは?
MAGEは、Multi Axis Generative Engineの略称です。
その名の通り、多軸でのGコードを生成する処理を表しています。

# 使用方法
MAGE Simulatorは、コマンドラインアプリです。
計算にCUDAを使用し、描画にopenGLを使用しています。
なお、並列計算を実行している際は、openGLでの描画はできないようになっています。

# 開発環境
+ Visual Studio Community 2022 (17.10.5)

## 使用ライブラリ
+ nupengl.core:バーション0.1.0.1
+ CUDA:バージョン12.6.68

## 環境設定方法
openGLは、NuGetでインストールすることができます。
CUDAは、NVIDIAの資料を参照しインストールしてください。

## コマンド
MAGE Simulatorは3個の引数を順番に渡す必要があります。
1. machine.xml(3Dプリンタの各種緒言・運動学などが定義されたファイル)のパス
2. 衝突検出の有無:True/False
3. マルチスレッド:True/False
