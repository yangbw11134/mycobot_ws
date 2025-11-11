// Master_code.cpp
#include <cstdlib>
#include <iostream>

int main() {
    // ====== 환경 변수 ======
    const char* ROBOT_USER = "cocobot";
    const char* ROBOT_IP   = "192.168.0.122";
    const char* REMOTE_PICS = "/home/cocobot/Pictures/snap*.jpg"; 
    const char* LOCAL_DIR  = "/home/kangmg/Downloads/";               
    // =========================

    // 1) 로봇에서 snap_move_and_snap 실행
    std::string cmd1 =
        "ssh cocobot@192.168.0.122 "
        "'bash -lc \""
        "export ROS_DOMAIN_ID=20; "
        "cd ~/cobot_ws; "
        "source /opt/ros/humble/setup.bash && "
        "source ~/cobot_ws/install/setup.bash && "
        "ros2 run moveit_joint_stepper_cpp snap_move_and_snap"
        "\"'";

    // 2) 촬영된 사진을 로컬로 가져오기
    std::string cmd2 = std::string("scp ") + ROBOT_USER + "@" + ROBOT_IP + ":" +
        REMOTE_PICS + " " + LOCAL_DIR;

    // 3) 로컬에서 sample_inspect.py 실행
    //    필요에 따라 파일 경로 수정 가능
    std::string cmd3 =
        "bash -lc \""
        "cd ~/mycobot_ws && "
        "source /opt/ros/humble/setup.bash && "
        "source ~/mycobot_ws/install/setup.bash && "
        "python3 sample_inspect.py "
        "--image ~/mycobot_ws/capture_sample.jpg "
        "--mask  ~/mycobot_ws/sam_output/sample_mask.png "
        "--homography_yaml ~/mycobot_ws/homography.yaml "
        "--compare_to    ~/mycobot_ws/master_ref/master_features.json "
        "--out_dir       ~/mycobot_ws/sample_result "
        "\"";

    // ===== 실행 단계 =====
    std::cout << "[run] " << cmd1 << std::endl;
    if (std::system(cmd1.c_str()) != 0) {
        std::cerr << "[err] 로봇 명령 실행 실패\n";
        return 1;
    }

    std::cout << "[run] " << cmd2 << std::endl;
    if (std::system(cmd2.c_str()) != 0) {
        std::cerr << "[err] scp 실패\n";
        return 2;
    }

    std::cout << "[run] " << cmd3 << std::endl;
    if (std::system(cmd3.c_str()) != 0) {
        std::cerr << "[err] 검사 스크립트 실행 실패\n";
        return 3;
    }

    std::cout << "[ok] 전체 파이프라인 완료!\n";
    return 0;
}
