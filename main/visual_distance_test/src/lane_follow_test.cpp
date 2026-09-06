#include "xtnetrc_runtime/app.hpp"
#include <iostream>
int main(int argc, char** argv) {
    std::cerr << "兼容入口：已切换到统一运行程序；参数见 --help。\n";
    return xtnetrc::runtime::run(argc, argv);
}
