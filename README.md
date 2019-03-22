# Spectral_Style_Transfer
学部での研究(2017/12/20最終更新)

モーションデータ(bvh形式)からスタイル(感情，個性)を抽出し，他のデータへと転写する．

用意するモーション
sorce     :動作Aスタイルあり
reference :動作Aスタイルなし
target    :動作Bスタイルなし

出力
output    :動作Bスタイルあり

sorceとreferenceが同一動作であるため，各モーションの関節角度の時間変化の波形をフーリエ変換し，位相と振幅の差分をスタイルとして抽出する．
その後同様にFFTしたtargetにスタイルを付与し，最適化処理後にIFFTする．

モーションの時間変化のタイミング調整には動的時間変化を用いている．
