# VT simulator  
## sim_node
### subscriber

|トピック名|機能|結果|
|:----|:---|:----|
|X1|キャプチャをシミュレートします。カメラ位置と視野範囲(ローカルパラメータ)から撮影される点群をパブリッシュします|Y1:撮影結果<br>ps_floats:点群|
|place|パラメータで指定した積み方でワークを配置します。|error:ワーク残数＞０で呼ばれた場合|
|place0|ワーク１個をワールド原点に配置します||
|pick1|両端以外のワークをピックします。ピックするワークは指定フレーム(デフォルトcamera/capture0/solve)を参照します。ピック可能なワークは、最上段の両端以外のワークです。それ以外のワークをピックしようとしたときはエラーとなります。|error:所定のワーク以外を取ろうとした。|
|pick2|両端のワークをピックします。このトピックでピック可能なワークは、最上段の両端ワークです。それ以外のワークをピックしようとしたときはエラーとなります。|error:所定のワーク以外を取ろうとした。|

### publisher

### 構成パラメータ

### パラメータ