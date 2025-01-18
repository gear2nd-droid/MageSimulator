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
- CoreXY-BC:ヘッドを支持するXYはCoreXYで動き、昇降するZ軸の上にB軸が、B軸の上にC軸がのり、C軸に造形ベッドが付いている機構。
- CoreXY:ヘッドを支持するXYはCoreXYで動き、昇降するZ軸に造形ベッドが付いている機構。
- BedSlingerY:ヘッドを支持するXはZの上に付いており、Yの上に造形ベッドが付いている機構。
- Delta:3軸の鉛直直動機構を用いたパラレルリンクでXYZを動作する機構。

### Changerタグ
3Dプリンタ自体の機械的な緒言を定義しています。
- FilamentDiameter:フィラメントの直径
- NozzleDiameter:ノズルの直径
- OriginX:原点のX座標
- OriginY:原点のY座標
- OriginZ:原点のZ座標
- HoppingDistance:XYZの移動量がこの値よりも大きいとZホップする
- HoppingDegreeA:Aの移動量がこの値よりも大きいとZホップする
- HoppingDegreeB:Bの移動量がこの値よりも大きいとZホップする
- HoppingDegreeC:Cの移動量がこの値よりも大きいとZホップする
- Type:運動学の計算方法の種類を選択する

#### CoreXY-BCのType
- For5x:5軸での運動学を正確に解いているもの。XYZBC軸全てが動作する際はこのモードを選択しないと、ノズルの法線方向が正確に制御されない。ただし、Zホップの回数が増えてしまい造形時間がかかり、また姿勢の正負が入れ替わる箇所で造形が途切れてしまう欠点がある。
- For4x:4軸での運動学を解いているもの。XYZB軸の動作のみを考慮しており、C軸が制御対象に加わると間違った運動学の解となり、ノズルの法線方向が正確に制御されなくなる。利点として、B軸が正負両方ともに値をとれるようになり、連続した造形が可能であり、Zホップの回数が減る利点がある。
- Normal:For5xと同様。

#### CoreXYのType
- Normal:3軸なため、Normalしかない。

#### BedSlingerYのType
- Normal:3軸なため、Normalしかない。

#### DeltaのType
- Normal:3軸なため、Normalしかない。

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

## MAGE Simulatorのマウス操作
MAGE Simulatorでは、マウス操作・キーボード操作で描画している状態を操作できます。
- 左ドラッグ:視点移動
- Shift+上下に左ドラッグ:ズームイン・ズームアウト
- 右ドラッグ:注視点移動
- 右矢印キー:次のレイヤーIDに移動
- 左矢印キー:前のレイヤーIDに移動
- ESCキー:指定した時間に移動(時間とGコードの行数は一致しません。Gコードの間を微細に補間するようにシミュレーションの時間を時分割しているためです。)

# 開発環境
+ Visual Studio Community 2022 (17.10.5)

## 使用ライブラリ
+ nupengl.core:バーション0.1.0.1
+ CUDA:バージョン12.6.68

## 環境設定方法
openGLは、NuGetでインストールすることができます。
CUDAは、NVIDIAの資料を参照しインストールしてください。
なお、CUDAは必須です。
