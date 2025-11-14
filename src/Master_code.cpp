
// /home/yangbi/mycobot_ws/src/mycobot_master_control/src/Master_code

// Master_code.cpp
#include <cstdlib>
#include <iostream>
#include <string>

int main() {
    // ====== 환경 변수 (필요 시 여기만 수정) ======
    const char* ROBOT_USER = "cocobot";
    const char* ROBOT_IP   = "192.168.0.122";
    const char* REMOTE_PICS = "'/home/cocobot/Pictures/snap_for_pca.jpg'"; // 원격 저장 경로 패턴
    const char* LOCAL_DIR  = "/home/yangbi/mycobot_ws/src/mycobot_master_control"; // 내 노트북 저장 폴더

    // 패키지 기본 경로
    const std::string PKG_PATH = "/home/yangbi/mycobot_ws/src/mycobot_master_control";
    const std::string RESOURCE_DIR = PKG_PATH + "/resource";
    const std::string CONFIG_DIR = PKG_PATH + "/config";
    const std::string SCRIPTS_DIR = PKG_PATH + "/scripts";
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

    // 3-1) SAM 처리 - 촬영된 이미지를 마스킹
    std::string cmd3 =
        "bash -lc \""
        "cd " + PKG_PATH + " && "
        "python3 scripts/single_sam.py "
        "--image " + RESOURCE_DIR + "/snap2.jpg "
        "--output " + RESOURCE_DIR + "/sample_mask.png "
        "--sam /home/yangbi/models/sam/sam_vit_h_4b8939.pth "
        "--roi-width 0.8 "
        "--roi-height 0.8 "
        "--roi-center-x 0.5 "
        "--roi-center-y 0.5 "
        "--save-debug"
        "\"";

    // 3-2) 로컬에서 sample_inspect.py 실행
    //    필요에 따라 파일 경로 수정 가능
    std::string cmd3_2 =
        "bash -lc \""
        "cd ~/mycobot_ws && "
        "source /opt/ros/humble/setup.bash && "
        "source ~/mycobot_ws/install/setup.bash && "
        "python3 " + SCRIPTS_DIR + "/sample_inspect.py "
        "--image " + RESOURCE_DIR + "/snap2.jpg "
        "--mask  " + RESOURCE_DIR + "/sample_mask.png "
        "--homography_yaml " + CONFIG_DIR + "/homography.yaml "
        "--camera_info " + CONFIG_DIR + "/camera_info.yaml "
        "--compare_to " + RESOURCE_DIR + "/master_ref/master_features.json "
        "--out_dir " + RESOURCE_DIR + "/sample_result " // <-- 마지막 공백 유지
        "\"";

    // 4) 집어서 스캔판으로 옮기기(go_to_scan.py)
    std::string cmd4 =
        std::string("ssh ") + ROBOT_USER + "@" + ROBOT_IP +
        " 'cd ~/cobot_ws && python3 go_to_scan.py'";

    // 5-1) 다시 컨베이어벨트로 (go_back_to_belt.py 실행 후 home 자세)
    std::string cmd5 =
        std::string("ssh ") + ROBOT_USER + "@" + ROBOT_IP +
        " 'cd ~/cobot_ws && python3 go_back_to_belt.py && python3 go_home.py'";

    // 5-2) 갖다버리기 (go_to_hell.py 실행 후 home 자세)
    std::string cmd6 =
        std::string("ssh ") + ROBOT_USER + "@" + ROBOT_IP +
        " 'cd ~/cobot_ws && python3 go_to_hell.py && python3 go_home.py'";

    // ===== 순차 실행 시작 =====

    // 1) 기본 포즈 + 사진 촬영
    std::cout << "[run] " << cmd1 << std::endl;
    int ret1 = std::system(cmd1.c_str());
    if (ret1 != 0) {
        std::cerr << "[err] 원격 실행 실패 (code=" << ret1 << ")\n";
        return 1;
    }

    // 2) 사진 scp
    std::cout << "[run] " << cmd2 << std::endl;
    int ret2 = std::system(cmd2.c_str());
    if (ret2 != 0) {
        std::cerr << "[err] 파일 가져오기 실패 (code=" << ret2 << ")\n";
        return 2;
    }

    // 3-1) SAM 처리
    std::cout << "[run] " << cmd3 << std::endl;
    if (std::system(cmd3.c_str()) != 0) {
        std::cerr << "[err] SAM 마스킹 실패\n"; return 3;
    }

    // 3-2) 로컬 검사 스크립트 실행
    std::cout << "[run] " << cmd3_2 << std::endl;
    int ret3_inspect_raw = std::system(cmd3_2.c_str());

    // 3-3) 1차 불량 여부 "자동" 판별
    int firstCheckResult = -1; // -1: 알 수 없음, 0: 정상, 1: 불량

    if (WIFEXITED(ret3_inspect_raw)) {
        firstCheckResult = WEXITSTATUS(ret3_inspect_raw); // 파이썬의 exit() 값
    } else {
        std::cerr << "[err] 검사 스크립트 비정상 종료 (code=" << ret3_inspect_raw << ")\n";
        return 3;
    }

    // firstCheckResult가 0이면 "pass"(정상), 1이면 "fail"(불량)
    if (firstCheckResult == 0) { 
        std::cout << "[info] 1차 판정: 정상(Pass). 프로그램을 종료합니다.\n";
        return 0;
    } else if (firstCheckResult == 1) {
        std::cout << "[info] 1차 판정: 불량(Fail). 스캔판으로 이동합니다.\n";
        // 불량이므로 다음 단계(4)로 계속 진행
    } else {
        std::cerr << "[err] 알 수 없는 검사 스크립트 반환값: " << firstCheckResult << "\n";
        return 3;
    }

    // 4) 불량이면 스캔판으로 이동
    std::cout << "[run] " << cmd4 << std::endl;
    int ret4 = std::system(cmd4.c_str());
    if (ret4 != 0) {
        std::cerr << "[err] go_to_scan 실행 실패 (code=" << ret4 << ")\n";
        return 4;
    }

    // 4-1) (스캔 자동 수행은 생략) 스캔/추가 검사 결과 수동 입력
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