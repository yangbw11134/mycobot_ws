// Master_code.cpp
#include <cstdlib>
#include <iostream>
#include <string>

int main() {
    // ====== 환경 변수 (필요 시 여기만 수정) ======
    const char* ROBOT_USER = "cocobot";
    const char* ROBOT_IP   = "192.168.0.122";
    const char* REMOTE_PICS = "'/home/cocobot/Pictures/snap_for_pca.jpg'"; // 원격 저장 경로 패턴
    const char* LOCAL_DIR  = "/home/kangmg/Downloads/";            // 내 노트북 저장 폴더
    // ============================================

    // 0) 컨베이어벨트 이동 (추후 필요 시 cmd0 추가)

    // 1) 로봇에서 ROS 환경 로드 후 기본 포즈로 이동, 이후 사진 찍기(go_home.py 안에서 처리)
    std::string cmd1 =
        std::string("ssh ") + ROBOT_USER + "@" + ROBOT_IP + R"( 'bash -lc "
            set -e;
            export ROS_DOMAIN_ID=20;
            cd ~/cobot_ws;
            source /opt/ros/humble/setup.bash;
            source ~/cobot_ws/install/setup.bash;
            python3 go_home.py;
            ./snap_cam --dev /dev/video0 --out ~/Pictures/snap_for_pca.jpg;
        "')";

    // 2) 원격 사진을 내 노트북으로 가져오기
    std::string cmd2 =
        std::string("scp ") + ROBOT_USER + "@" + ROBOT_IP + ":" + REMOTE_PICS + " " + LOCAL_DIR;


    // 3) SAM처리 후 1차불량판별, 결과 출력

    // 4) 집어서 스캔판으로 옮기기(go_to_scan.py)
    std::string cmd4 =
        std::string("ssh ") + ROBOT_USER + "@" + ROBOT_IP +
        " 'cd ~/cobot_ws && python3 go_to_scan.py'";

    // 5) 스캔 후 2차불량판별, 결과출력

    // 5-1) 다시 컨베이어벨트로 (go_back_to_belt.py 실행 후 home 자세)
    std::string cmd5 =
        std::string("ssh ") + ROBOT_USER + "@" + ROBOT_IP +
        " 'cd ~/cobot_ws && python3 go_back_to_belt.py && python3 go_home.py'";

    // 5-2) 갖다버리기 (go_to_hell.py 실행 후 home 자세)
    std::string cmd6 =
        std::string("ssh ") + ROBOT_USER + "@" + ROBOT_IP +
        " 'cd ~/cobot_ws && python3 go_to_hell.py && python3 go_home.py'";




    
    // ===== 순차 실행 시작 =====
    

    // 0) 컨베이어벨트 이동

    // 1) 기본 포즈 + 사진 촬영
    std::cout << "[run] " << cmd1 << std::endl;
    int ret1 = std::system(cmd1.c_str());
    if (ret1 != 0) {
        std::cerr << "[err] 원격 실행 실패 (code=" << ret1 << ")\n";
        return 1;
    }

    // 2) 사진 scp로 불러오기
    std::cout << "[run] " << cmd2 << std::endl;
    int ret2 = std::system(cmd2.c_str());
    if (ret2 != 0) {
        std::cerr << "[err] 파일 가져오기 실패 (code=" << ret2 << ")\n";
        return 2;
    }

    // 3) 사진 SAM 처리 후 1불량여부 출력

    // 3-1) 1차 불량 여부 수동입력
    char firstCheck;
    std::cout << "[질문] 1차 판정에서 불량인가요? (y/n): ";
    std::cin >> firstCheck;

    if (firstCheck == 'n' || firstCheck == 'N') {
        std::cout << "[info] 정상품으로 판정됨. 프로그램을 종료합니다.\n";
        return 0;
    }

    // 4) 1차 불량이면 스캔판으로 이동
    std::cout << "[run] " << cmd4 << std::endl;
    int ret4 = std::system(cmd4.c_str());
    if (ret4 != 0) {
        std::cerr << "[err] go_to_scan 실행 실패 (code=" << ret4 << ")\n";
        return 4;
    }

    // 4-1) 스캔 후 2차불량여부 출력

    // 5) (스캔 자동 수행은 생략) 스캔/추가 검사 결과 수동 입력
    char secondCheck;
    std::cout << "[질문] 스캔/추가 검사 결과는 어떤가요? (g: 정상→컨베이어, b: 불량→폐기): ";
    std::cin >> secondCheck;

    if (secondCheck == 'g' || secondCheck == 'G') {
        // 5-1) 다시 컨베이어벨트로
        std::cout << "[run] " << cmd5 << std::endl;
        int ret5 = std::system(cmd5.c_str());
        if (ret5 != 0) {
            std::cerr << "[err] go_back_to_belt 실행 실패 (code=" << ret5 << ")\n";
            return 5;
        }
    } else if (secondCheck == 'b' || secondCheck == 'B') {
        // 5-2) 갖다버리기
        std::cout << "[run] " << cmd6 << std::endl;
        int ret6 = std::system(cmd6.c_str());
        if (ret6 != 0) {
            std::cerr << "[err] go_to_hell 실행 실패 (code=" << ret6 << ")\n";
            return 6;
        }
    } else {
        std::cerr << "[err] 알 수 없는 입력값입니다. (입력: " << secondCheck << ")\n";
        return 7;
    }

    std::cout << "[ok] 전체 시퀀스 완료.\n";
    return 0;
}
