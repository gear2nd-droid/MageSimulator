# はじめに
[MAGE Slicer](https://github.com/gear2nd-droid/MageSlicer)と共に用いて多軸3Dプリンタを動作させるためのソフトウェア群です。

## MAGE Simulator
MAGE Simulatorは、Gコードと対象とする3Dプリンタに関する情報を読み込み、動作をシミュレートするソフトウェアです。
MAGE Simulatorでは、動作をシミュレートする際に、造形物と3Dプリンタ自体の衝突検出を行うことができます。
一般的な3軸の3Dプリンタは、水平面を鉛直方向に積層していく動作をするだけなため、干渉することは起こりえません。
しかしながら、多軸の3Dプリンタでは動作が複雑になり、既に造形された部分にヘッドが動いたり、造形ベッドとヘッドが干渉するなどの機械的な干渉が起こりえます。
そのため、多軸の3Dプリンタでは衝突検出が必要になります。

## MAGE Interface
MAGE Interfaceは、MAGE Slicerでスライスした中間ファイルを3Dプリンタを動かすためのGコードに変換します。
MAGE InterfaceでのGコードへの変換では、3Dプリンタ自体の運動学を考慮して変換します。
また、MAGE Interfaceでは、MAGE Simulatorを起動することができます。

## MAGEとは?
MAGEは、Multi Axis Generative Engineの略称です。
その名の通り、多軸でのGコードを生成する処理を表しています。

# 使用方法
MAGE Interfaceは、Windowsアプリです。
MAGE Slicerの中間ファイル(csv)と3Dプリンタの緒言の設定ファイル(xml)を読み込み動作します。
なお、Gコードへと変換するための緒言も設定ファイルとして別途保存し、読み込むことができます。

MAGE Simulatorは、コマンドラインアプリです。
計算にCUDAを使用し、描画にopenGLを使用しています。
なお、並列計算を実行している際は、openGLでの描画はできないようになっています。

## MAGE Interfaceの設定(machine.xml)
現在対応している機構はCoreXY-BCのみです。

### Kinematicsタグ
対象としている機構を指定します。この機構を参照し、運動学を解き中間ファイルを変換します。
- CoreXY-BC

### Changerタグ
3Dプリンタ自体の機械的な緒言を定義しています。
- FilamentDiameter:フィラメントの直径
- NozzleDiameter:ノズルの直径
- OriginX:原点のX座標
- OriginY:原点のY座標
- OriginZ:原点のZ座標
- HoppingDistance:移動量がこの値よりも大きいとZホップする

### Headsタグ
ノズルを含めたヘッドの構成要素を定義します。
複数のHeadタグを含めることができます。
各HeadタグはBoxまたはCylinderを選択することができます。

### XGantrysタグ
ヘッドを支持しているX方向のガントリーの構成要素を定義します。
複数のXGantryタグを含めることができます。
各XGantryタグはBoxまたはXylinderを選択することができます。

### YGantrysタグ
ヘッドを支持しているY方向のガントリーの構成要素を定義します。
複数のYGantryタグを含めることができます。
各YGantryタグはBoxまたはCylinderを選択することができます。

### Bedsタグ
造形ベッドの構成要素を定義します。
複数のBedタグを含めることができます。
各BedタグはBoxまたはCylinderを選択することができます。

### Box構成要素
- Type:Box
- CenterX:直方体の中央のX座標
- CenterY:直方体の中央のY座標
- CenterZ:直方体の中央のZ座標
- SizeX:直方体のX方向の大きさ
- SizeY:直方体のY方向の大きさ
- SizeZ:直方体のZ方向の大きさ

### Cylinder構成要素
- Type:Cylinder
- LowerRadius:円柱の下面の半径
- UpperRadius:円柱の上面の半径
- Height:円柱の高さ
- Div:円弧の分割数
- LowerX:円柱の下面のX座標
- LowerY:円柱の下面のY座標
- LowerZ:円柱の下面のZ座標

※Cylinderでは円柱の上面と下面の半径を異ならせることができます。
これによって、ノズル形状などを模擬することができます。

## MAGE Simulatorのコマンド
MAGE Simulatorは3個の引数を順番に渡す必要があります。
1. machine.xml(3Dプリンタの各種緒言・運動学などが定義されたファイル)のパス
2. 衝突検出の有無:True/False
3. マルチスレッド:True/False

※なお、MAGE InterfaceからMAGE Simulatorを相対リンクで起動しています。

# 開発環境
+ Visual Studio Community 2022 (17.10.5)

## 使用ライブラリ
+ nupengl.core:バーション0.1.0.1
+ CUDA:バージョン12.6.68

## 環境設定方法
openGLは、NuGetでインストールすることができます。
CUDAは、NVIDIAの資料を参照しインストールしてください。
なお、CUDAは必須です。
