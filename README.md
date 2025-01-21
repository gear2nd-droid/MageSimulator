**日本語は下にあります。**

# Introduction
Software suite for operating a multi-axis 3D printer with [MAGE Slicer](https://github.com/gear2nd-droid/MageSlicer).

## MAGE Simulator
MAGE Simulator is software that reads G-code and information about the target 3D printer and simulates its operation.
MAGE Simulator can detect collisions between the object and the 3D printer itself when simulating motion.
A typical 3-axis 3D printer only works by stacking horizontal surfaces in a vertical direction, so collisions are not likely to occur.
However, multi-axis 3D printers have a more complex operation, and mechanical interference can occur, such as the head moving over an already formed part or interference between the bed and the head.
Therefore, multi-axis 3D printers require collision detection.

## MAGE Interface
MAGE Interface converts the intermediate files sliced by MAGE Slicer into G-code for running the 3D printer.
The conversion to G-code in MAGE Interface takes into account the kinematics of the 3D printer itself.
The MAGE Interface can also launch the MAGE Simulator.

## What is MAGE?
MAGE stands for Multi Axis Generative Engine.
As the name implies, it represents the process of generating G-code in multiple axes.

## MAGE Printer
![DSC09175](https://github.com/user-attachments/assets/b4bbf459-22e9-47bb-ad0a-abf2bc8486a3)
MAGE Printer is a 5-axis 3D printer compatible with MAGE Interface and MAGE Simulator. A book on how to produce it is available at [Kindle Direct Publishing](https://www.amazon.com/dp/B0DTGR7GW4).

# Usage rules
MAGE Interface is a Windows application.
It reads and works with MAGE Slicer intermediate files (csv) and the 3D printer's verbal settings file (xml).
The message to be converted to G-code can also be saved separately as a configuration file and read in.

MAGE Simulator is a command line application.
It uses CUDA for computation and openGL for rendering.
Note that drawing with openGL is not available when parallel computation is running.

## MAGE Interface operation settings (set via GUI)
- Nozzle temperature
- Bed temperature
- Printing speed for exterior and interior walls (1st layer and beyond)
- Printing speed of infill (1st layer and beyond)
- Printing speed of support (1st layer and beyond)
- Travel speed without printing (two types: first layer and beyond)
- Output of molding cooling fan (two types: first layer and beyond)
- Retract Length
- Retraction pullback speed
- Retractor extrusion speed
- Z-hop height
- Select with or without collision detection
- Multi-threaded use selection

## Machine settings for MAGE Interface (machine.xml)
The MAGE Interface reads an XML file to configure the 3D printer itself.
The configuration items are as follows

### Kinematics Tags
Specify the mechanism you are targeting. Refer to this mechanism, solve the kinematics and convert the intermediate file to G-code.
- CoreXY-BC:The XY axis that supports the head moves with CoreXY, the B axis is on top of the Z axis, which is elevated, the C axis is on top of the B axis, and the molding bed is attached to the C axis.
- CoreXY:The XY that supports the head moves with CoreXY, and the mechanism with the molding bed on the Z axis that raises and lowers.
- BedSlingerY:The X that supports the head is attached above the Z, and the mechanism with the molding bed above the Y.
- Delta:A mechanism that operates in XYZ with a parallel link using a 3-axis vertical linear motion mechanism.

### Changer tag
The 3D printer itself defines the mechanical syllabus of the 3D printer.
- FilamentDiameter:Filament diameter
- NozzleDiameter:Nozzle diameter
- OriginX:X coordinate of origin
- OriginY:Y coordinate of origin
- OriginZ:Z coordinate of origin
- HoppingDistance:Z-hop if the amount of XYZ movement is greater than this value.
- HoppingDegreeA:Z-hop if the amount of movement of A is greater than this value.
- HoppingDegreeB:Z-hop if the amount of movement of B is greater than this value.
- HoppingDegreeC:Z-hop if the amount of movement of C is greater than this value.
- Type:Select the type of kinematics calculation method

#### Type of CoreXY-BC
- For5x:This is a precise solution of kinematics in 5 axes; when all XYZBC axes operate, the nozzle normal direction is not accurately controlled unless this mode is selected. However, the drawback is that the number of Z-hops increases, which takes time for modeling, and modeling is interrupted at the point where the positive and negative postures are switched.
- For4x:It solves for 4-axis kinematics, considering only XYZB-axis motion; if C-axis is added to the control target, the wrong kinematics will be solved and the nozzle normal direction will not be accurately controlled. The advantage is that the B-axis can take both positive and negative values, allowing for continuous modeling and reducing the number of Z-hops.
- Normal:Similar to For5x.

#### Type of CoreXY
- Normal:Since it is triaxial, only Normal is available.

#### Type of BedSlingerY
- Normal:Since it is triaxial, only Normal is available.

#### Type of Delta
- Normal:Since it is triaxial, only Normal is available.

### Heads tag
Defines the components of the head, including the nozzle.
Multiple Head tags can be included.
Each Head tag can be Box or Cylinder.

### XGantrys Tags
Defines the components of the gantry in the X direction that support the head.
Multiple XGantry tags can be included.
Each XGantry tag can be Box or Cylinder.

### YGantrys Tags
Defines the components of the gantry in the Y direction that support the head.
Multiple YGantry tags can be included.
Each YGantry tag can be Box or Cylinder.

### Beds tag
Defines the components of a molding bed.
Multiple Bed tags can be included.
Each Bed tag can be a Box or a Cylinder.

### Box Components
- Type:Box
- CenterX:X coordinate of the center of the rectangle
- CenterY:Y coordinate of the center of the rectangle
- CenterZ:Z coordinate of the center of the rectangle
- SizeX:Size of rectangular body in x-direction
- SizeY:Size of rectangular body in y-direction
- SizeZ:Size of rectangular body in z-direction

### Cylinder components
- Type:Cylinder
- LowerRadius:Radius of the underside of the cylinder
- UpperRadius:Radius of the top surface of the cylinder
- Height:Height of cylinder
- Div:Number of arc divisions
- LowerX:X-coordinate of the bottom surface of the cylinder
- LowerY:Y-coordinate of the bottom surface of the cylinder
- LowerZ:Z-coordinate of the bottom surface of the cylinder

※Cylinder allows the radii of the top and bottom surfaces of a cylinder to be different.
This allows for the simulation of nozzle shapes, etc.

## MAGE Simulator Commands
MAGE Simulator requires three arguments to be passed in sequence.
1. Path to machine.xml (a file that defines the various instructions, kinematics, etc. of the 3D printer)
2. Collision detection: True/False
3. Multithreading: True/False

※Note that MAGE Simulator is started from MAGE Interface with a relative link.

## MAGE Simulator Operation
In MAGE Simulator, you can manipulate the state being drawn with mouse and keyboard operations.
- Left drag: Move viewpoint
- Shift+drag up/down left: zoom in/out
- Right drag: Move gazing point
- Right arrow key: Move to next layer ID
- Left arrow key: Go to previous layer ID
- ESC key followed by a numerical value: move to the specified time (the time and the number of lines of G-code do not match, because the simulation time is divided by time so that the G-code is finely interpolated between the G-code).

# Development environment
+ Visual Studio Community 2022 (17.10.5)

## Libraries used
+ nupengl.core:Version 0.1.0.1
+ CUDA:Version 12.6.68

## How to set up the environment
openGL can be installed with NuGet.
For CUDA, please refer to the NVIDIA documentation for installation.
CUDA is required.

**The rest is in Japanese.**

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

## MAGE Printer
![DSC09175](https://github.com/user-attachments/assets/b4bbf459-22e9-47bb-ad0a-abf2bc8486a3)
MAGE Printerは、MAGE InterfaceとMAGE Simulatorに対応した5軸3Dプリンタです。 [Kindle Direct Publishing](https://www.amazon.com/dp/B0DTGR7GW4)にて、製作方法をまとめた書籍を販売しております。

# 使用方法
MAGE Interfaceは、Windowsアプリです。
MAGE Slicerの中間ファイル(csv)と3Dプリンタの緒言の設定ファイル(xml)を読み込み動作します。
なお、Gコードへと変換するための緒言も設定ファイルとして別途保存し、読み込むことができます。

MAGE Simulatorは、コマンドラインアプリです。
計算にCUDAを使用し、描画にopenGLを使用しています。
なお、並列計算を実行している際は、openGLでの描画はできないようになっています。

## MAGE Interfaceの動作設定(GUIにて設定)
- ノズル温度
- ベッド温度
- 外壁・内壁の印刷速度(1層目とそれ以降の2種類)
- インフィルの印刷速度(1層目とそれ以降の2種類)
- サポートの印刷速度(1層目とそれ以降の2種類)
- 印刷無しの移動速度(1層目とそれ以降の2種類)
- 造形物冷却ファンの出力(1層目とそれ以降の2種類)
- リトラクトの長さ
- リトラクトの引き戻し速度
- リトラクトの押し出し速度
- Zホップの高さ
- 衝突検出の有無選択
- マルチスレッドの使用選択

## MAGE Interfaceの機械設定(machine.xml)
MAGE Interfaceでは、XMLファイルを読み込んで3Dプリンタ自体の設定を行います。
設定項目は次のようになっています。

### Kinematicsタグ
対象としている機構を指定します。この機構を参照し、運動学を解き中間ファイルをGコードに変換します。
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
各XGantryタグはBoxまたはCylinderを選択することができます。

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

## MAGE Simulatorの操作
MAGE Simulatorでは、マウス操作・キーボード操作で描画している状態を操作できます。
- 左ドラッグ:視点移動
- Shift+上下に左ドラッグ:ズームイン・ズームアウト
- 右ドラッグ:注視点移動
- 右矢印キー:次のレイヤーIDに移動
- 左矢印キー:前のレイヤーIDに移動
- ESCキーの後数値入力:指定した時間に移動(時間とGコードの行数は一致しません。Gコードの間を微細に補間するようにシミュレーションの時間を時分割しているためです。)

# 開発環境
+ Visual Studio Community 2022 (17.10.5)

## 使用ライブラリ
+ nupengl.core:バーション0.1.0.1
+ CUDA:バージョン12.6.68

## 環境設定方法
openGLは、NuGetでインストールすることができます。
CUDAは、NVIDIAの資料を参照しインストールしてください。
なお、CUDAは必須です。
