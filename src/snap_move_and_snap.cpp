#include <cstdlib>   // system()
#include <iostream>

int main() {
    // 1. 첫 번째 촬영
    const char* cmd1 = "/home/cocobot/cobot_ws/snap_cam --dev /dev/video0 --out ~/Pictures/snap1.jpg";
    // 2. 로봇 움직이기
    const char* cmd2 = "ros2 run moveit_joint_stepper_cpp move_joint_delta --group arm_group";
    // 3. 두 번째 촬영
    const char* cmd3 = "/home/cocobot/cobot_ws/snap_cam --dev /dev/video0 --out ~/Pictures/snap2.jpg";
    // 4. 두 장 전송 (scp 사용)
    const char* cmd4 = "scp -q ~/Pictures/snap1.jpg ~/Pictures/snap2.jpg kangmg@192.168.0.149:~/Downloads/";

    std::cout << "[run] " << cmd1 << std::endl;
    int ret1 = std::system(cmd1);
    if (ret1 != 0) std::cerr << "[err] 첫 번째 명령어 실패 (code=" << ret1 << ")\n";

    std::cout << "[run] " << cmd2 << std::endl;
    int ret2 = std::system(cmd2);
    if (ret2 != 0) std::cerr << "[err] 두 번째 명령어 실패 (code=" << ret2 << ")\n";

    std::cout << "[run] " << cmd3 << std::endl;
    int ret3 = std::system(cmd3);
    if (ret3 != 0) std::cerr << "[err] 세 번째 명령어 실패 (code=" << ret3 << ")\n";

    // 추가된 부분 ↓↓↓
    std::cout << "[run] " << cmd4 << std::endl;
    int ret4 = std::system(cmd4);
    if (ret4 != 0) std::cerr << "[err] 네 번째 명령어(파일 전송) 실패 (code=" << ret4 << ")\n";
    // ↑↑↑ 여기까지 추가

    return 0;
}


