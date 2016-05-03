# Deformation Transfer

論文 "Deformation Transfer for Triangle Meshes" [Robert Sumner et al. 2004] を実装した.　
http://people.csail.mit.edu/sumner/research/deftransfer/Sumner2004DTF.pdf

ざっくりと内容をまとめると,2枚のソース画像(undeformedな画像とdeformedな画像)におけるメッシュの変形をターゲット画像に適応し,ターゲットの変形後の画像を生成すればよい.

論文内の式変形では式変形が難しい箇所があるので Sumnerの博論3章を参照した.
http://people.csail.mit.edu/sumner/thesis/Sumner2005MMU.pdf

使用したdataは
- source undeformed　→　kawai_N.obj
- source deformed → kawai_a.obj
- target undeformed → mizo_N.obj
- target deformed → mizo_a.obj　　(mizo_N.objのコピーなので実装部分で上書きする)
