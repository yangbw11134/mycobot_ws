// Master_code.cpp
#include <cstdlib>
#include <iostream>

int main() {
    // ====== 환경 변수 (필요 시 여기만 수정) ======
    const char* ROBOT_USER = "cocobot";
    const char* ROBOT_IP   = "192.168.0.122";
    const char* REMOTE_PICS = "/home/cocobot/Pictures/snap*.jpg"; // 원격 저장 경로 패턴
    const char* LOCAL_DIR  = "/home/kangmg/Downloads/";           // 내 노트북 저장 폴더
    // ===========================================

    // 1) 로봇에서 ROS 환경 로드 후 snap_move_and_snap 실행
    std::string cmd1 =
 	 "ssh cocobot@192.168.0.122 "
 	 "'bash -lc \""
 	 "export ROS_DOMAIN_ID=20; "                      
  	  "cd ~/cobot_ws; "                                 
 	 "source /opt/ros/humble/setup.bash && "
 	 "source ~/cobot_ws/install/setup.bash && "
 	 "ros2 run moveit_joint_stepper_cpp snap_move_and_snap"
 	 "\"'";


    // 2) 실행 성공 시, 원격 사진을 내 노트북으로 가져오기
    std::string cmd2 = std::string("scp ") + ROBOT_USER + "@" + ROBOT_IP + ":" +
        REMOTE_PICS + " " + LOCAL_DIR;

    std::cout << "[run] " << cmd1 << std::endl;
    int ret1 = std::system(cmd1.c_str());
    if (ret1 != 0) {
        std::cerr << "[err] 원격 실행 실패 (code=" << ret1 << ")\n";
        return 1;
    }

    std::cout << "[run] " << cmd2 << std::endl;
    int ret2 = std::system(cmd2.c_str());
    if (ret2 != 0) {
        std::cerr << "[err] 파일 가져오기 실패 (code=" << ret2 << ")\n";
        return 2;
    }

    std::cout << "[ok] 완료! 사진이 " << LOCAL_DIR << " 로 복사되었습니다.\n";
    return 0;
}
