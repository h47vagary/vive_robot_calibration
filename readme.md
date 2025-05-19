## 编译步骤
本项目使用 QT 作为 UI 控件，默认开发者电脑装配了 QT5 环境，开发需要做如下修改才能编译此项目：
- 最外层 CMakeLists.txt 的 `set(CMAKE_PREFIX_PATH "D:/QT/5.12.9/mingw73_64/lib/cmake")` 需修改为开发者电脑上的对应路径
- 不使用 QT开发工具打包运行环境的话，则在运行时需要让程序知道QT库的对应路径，作者是直接添加环境变量的方式，下方贴出供参考：
  1. `D:\QT\5.12.9\mingw73_64\bin`
  2. `D:\QT\Tools\mingw730_64\bin`
