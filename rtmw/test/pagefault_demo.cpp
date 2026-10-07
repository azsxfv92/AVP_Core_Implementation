//
// W18 D1-E — pagefault_demo : "할당했다 = RAM 을 받았다" 가 아니라는 걸 숫자로 확인한다
//
// 빌드: g++ -std=c++17 -Wall -Wextra -O2 -g rtmw/test/pagefault_demo.cpp -o rtmw/build/pagefault_demo
// 실행: ./rtmw/build/pagefault_demo 256      (MiB 단위, 생략하면 256)
//
// 출력은 CSV — stage,pages_touched,minflt_delta,majflt_delta,rss_kb,rss_delta_kb
//   '#' 로 시작하는 줄은 설명 줄 (tools/stats.py 가 자동으로 무시한다)
//
// TODO 1~5 를 채운다. 뼈대 그대로도 빌드·실행은 된다 (값이 0 으로 나올 뿐).
// 막히면 docs/week18/d1_answers.md 를 열어라.
//

#include <cstddef>          // std::size_t
#include <cstdio>           // std::printf
#include <cstdlib>          // std::strtol, std::strtoul
#include <fstream>          // std::ifstream
#include <memory>           // std::unique_ptr, std::make_unique
#include <string>           // std::string, std::getline
#include <sys/resource.h>   // getrusage
#include <unistd.h>         // sysconf

// ============================================================================
// 측정 도구 — 완성본 (읽고 이해만 한다)
// ============================================================================

// /proc/self/status 에서 "VmRSS:" 줄을 찾아 kB 숫자를 돌려준다. 못 찾으면 -1.
// /proc/self = "지금 이 프로세스 자신"의 폴더 (셸에서 쓴 /proc/$$ 와 같은 역할)
long read_rss_kb() {
    std::ifstream f("/proc/self/status");
    std::string line;
    while (std::getline(f, line)) {                              // 한 줄씩 읽는다
        if (line.rfind("VmRSS:", 0) == 0) {                      // 줄이 "VmRSS:" 로 시작하면
            return std::strtol(line.c_str() + 6, nullptr, 10);   // 앞 6글자 뒤의 숫자만 꺼낸다
        }
    }
    return -1;
}

// ============================================================================
// TODO 영역
// ============================================================================

// TODO 1: 페이지 크기(바이트)를 커널에 물어서 돌려준다.
//         A-1 의 `getconf PAGESIZE` 와 같은 값이 나와야 한다.
//         힌트: sysconf(_SC_PAGESIZE)  — long 을 돌려준다
std::size_t page_size() {
    return static_cast<std::size_t>(sysconf(_SC_PAGESIZE));
}

// 이 프로세스에 지금까지 난 페이지 폴트의 누적 횟수
struct Faults {
    long minor = 0;   // RAM 안에서 해결된 것
    long major = 0;   // 디스크까지 다녀온 것
};

// TODO 2: getrusage 로 누적 폴트 횟수를 채운다.
//         힌트: struct rusage ru{};  getrusage(RUSAGE_SELF, &ru);
//               → ru.ru_minflt, ru.ru_majflt
Faults read_faults() {
    Faults f;
    struct rusage ru{};

    if(getrusage(RUSAGE_SELF, &ru) == 0){ // 커널이 내 프로세스에서 카운트하는 정보들을 받음(CPU, fault, time 등)
        f.minor = ru.ru_minflt;
        f.major = ru.ru_majflt;
    }
    return f;
}

// TODO 3: base[offset] 부터 len 바이트 구간에서, 페이지마다 딱 1바이트만 쓴다.
//         건드린 페이지 수를 돌려준다.
//         힌트: i 를 offset 부터 page 씩 늘리며 v[i] = 1;  touched 도 1 씩 늘린다
std::size_t touch_pages(char* base, std::size_t offset, std::size_t len, std::size_t page) {
    if (base == nullptr) return 0;   // 아직 할당 전(TODO 4 전)이면 아무것도 안 한다
    volatile char* v = base;         // volatile: "아무도 안 읽는 쓰기"를 컴파일러가 지우지 못하게
                                     //           (MCU 에서 레지스터 접근에 volatile 을 붙이는 것과 같은 이유)
    std::size_t touched = 0;
    for (std::size_t i = offset; i < offset + len; i += page) {
        v[i] = 1;
        ++touched;
    }

    return touched;
}

// ============================================================================
// 보고 — 완성본
// ============================================================================

struct Snapshot {
    Faults f;
    long   rss_kb;
};

Snapshot snap() { return { read_faults(), read_rss_kb() }; }

// 한 단계의 결과를 CSV 한 줄로 찍는다. 폴트·RSS 는 "이 단계에서 늘어난 양"(after - before)이다.
void report(const char* stage, std::size_t pages_touched, const Snapshot& before, const Snapshot& after) {
    std::printf("%s,%zu,%ld,%ld,%ld,%ld\n",
                stage,
                pages_touched,
                after.f.minor - before.f.minor,
                after.f.major - before.f.major,
                after.rss_kb,
                after.rss_kb - before.rss_kb);
}

int main(int argc, char** argv) {
    const std::size_t mib   = (argc > 1) ? std::strtoul(argv[1], nullptr, 10) : 256;
    const std::size_t bytes = mib * 1024 * 1024;
    const std::size_t page  = page_size();

    std::printf("# pagefault_demo: %zu MiB = %zu pages of %zu bytes\n", mib, bytes / page, page);
    std::printf("stage,pages_touched,minflt_delta,majflt_delta,rss_kb,rss_delta_kb\n");

    Snapshot prev = snap();
    Snapshot now{};

    // [1] 할당만 한다 — 값은 채우지 않는다
    // TODO 4: bytes 크기의 char 배열을 unique_ptr 로 만든다. "값을 채우지 않는" 방식으로.
    //         힌트: std::unique_ptr<char[]> 에 new char[bytes] 를 넘긴다 (끝에 () 를 붙이지 않는다)
    std::unique_ptr<char[]> buf(new char[bytes]);
    std::printf("# alloc ptr=%p\n", static_cast<void*>(buf.get()));   // 주소를 밖으로 내보내야 컴파일러가 할당 자체를 없애지 못한다
    now = snap();  report("alloc", 0, prev, now);  prev = now;

    // [2] 앞 절반만 건드린다
    std::size_t t = touch_pages(buf.get(), 0, bytes / 2, page);
    now = snap();  report("touch_half", t, prev, now);  prev = now;

    // [3] 나머지 절반을 건드린다
    t = touch_pages(buf.get(), bytes / 2, bytes - bytes / 2, page);
    now = snap();  report("touch_rest", t, prev, now);  prev = now;

    // [4] 전부 한 번 더 — 이미 RAM 이 붙은 페이지를 다시 건드리면?
    t = touch_pages(buf.get(), 0, bytes, page);
    now = snap();  report("touch_again", t, prev, now);  prev = now;

    // [5] 반납 — unique_ptr 가 delete[] 를 부른다 (RAII)
    buf.reset();
    now = snap();  report("free", 0, prev, now);  prev = now;

    // [6] 보너스 — 같은 크기를 make_unique 로 만든다. 우리 코드는 한 바이트도 안 건드린다.
    // TODO 5: std::make_unique<char[]>(bytes) 로 만든다.
    std::unique_ptr<char[]> zeroed = std::make_unique<char[]>(bytes);   // TODO: 여기서 할당
    std::printf("# alloc_zeroed ptr=%p\n", static_cast<void*>(zeroed.get()));
    now = snap();  report("alloc_zeroed", 0, prev, now);  prev = now;

    return 0;
}
