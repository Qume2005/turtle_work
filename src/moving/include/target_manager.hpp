#ifndef TARGET_MANAGER_HPP
#define TARGET_MANAGER_HPP

#include <array>
#include <tuple>
#include <chrono>

using Target = std::tuple<double, double, double, std::chrono::duration<int64_t>>;

class TargetManager {
public:
    // 构造函数初始化列表
    TargetManager(Target& target)
        : target_(target) {}

    Target target_;
};

#endif // TARGET_MANAGER_HPP


