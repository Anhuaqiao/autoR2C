<h1 align="center">Welcome to 毫米波雷达-视觉-自动标定 👋</h1>
<p>
  <img alt="Version" src="https://img.shields.io/badge/version-1.0-blue.svg?cacheSeconds=2592000" />
</p>

> 此项目可以半自动标定毫米波雷达和相机

## Usage

```sh

图像数据处理（如果存在图像反转）
将所有图像数据放入 Pnpsolver_foruse/flip_pic/img_data 路径， 翻转后的结果存入 Pnpsolver_foruse/flip_pic/flipped。将翻转后的图片存入/Pnpsolver_foruse/solver/img_data。

雷达数据处理
需要在数据采集中确定目标的json数据编号。需要在/Pnpsolver_foruse/write_config/src/main.cpp源代码中修改向量 id 以及 file。id中每个元素存储在名为file中对应位置元素的.json文件中的目标id名称。将得到的config.yaml存入
/Pnpsolver_foruse/solver/radar_id。

变换矩阵求解
注意此时json和图片文件名需对应。
首次运行solver需要手动点出目标像素点。注意此处点击点击顺序需与write_config的代码中存储id相对应。首次运行后像素坐标会被保存在/Pnpsolver_foruse/solver/img_data以供下次直接使用。

本方法使用solvepnp对3D-2D对应点进行求解。

请在/Pnpsolver_foruse/solver/img_data_verification中存储验证的图片数据。验证结果会存储在/Pnpsolver_foruse/solver/test中。

本方法提供了BA方法，对solvepnp方法的结果进行调整。（尽管solvepnp结果已经是经过优化之后的，BA仍可作为验证）
BA的验证结果存储在/Pnpsolver_foruse/solver/test_ba

求解获得的旋转矩阵，旋转向量，平移向量以及欧拉角存储在/Pnpsolver_foruse/solver/results中。


```

## Author

👤 **汤笑寒**

* Github: [@Anhuaqiao](https://github.com/Anhuaqiao)
* LinkedIn: [@Xiaohan Tang](https://linkedin.com/in/xiaohan-tang-b3545118b/)

## Show your support

Give a ⭐️ if this project helped you!

<a href="https://www.patreon.com/Navinfo">
  <img src="https://c5.patreon.com/external/logo/become_a_patron_button@2x.png" width="160">
</a>

***
_This README was generated with ❤️ by [readme-md-generator](https://github.com/kefranabg/readme-md-generator)_