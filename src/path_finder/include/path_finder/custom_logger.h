#pragma once
#include <fstream>
#include <sstream>
#include <mutex>
#include <string>
#include <ctime>
#include <stdexcept>
#include <utility>

class CustomLogger {
public:
    explicit CustomLogger(const std::string& file_path) {
        file_.open(file_path, std::ios::out | std::ios::app);
        if (!file_.is_open()) {
            throw std::runtime_error("Failed to open log file: " + file_path);
        }
    }
    ~CustomLogger() { if (file_.is_open()) file_.close(); }

    class Stream;

    // 1) Use a named local so NRVO kicks in (no move/copy required)
    Stream stream() {
        Stream s(*this);
        return s;  // NRVO
    }

    void log(const std::string& msg) {
        const std::string ts = timestamp();
        std::lock_guard<std::mutex> lock(mtx_);
        file_ << "[" << ts << "] " << msg << '\n';
        file_.flush();
    }

private:
    friend class Stream;

    static std::string timestamp() {
        std::time_t now = std::time(nullptr);
        char buf[32];
        std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", std::localtime(&now));
        return buf;
    }

    std::ofstream file_;
    std::mutex mtx_;

public:
    class Stream {
    public:
        explicit Stream(CustomLogger& parent) : parent_(parent) {}

        // Non-copyable
        Stream(const Stream&) = delete;
        Stream& operator=(const Stream&) = delete;

        // 2) Explicit move ctor; keep the same parent ref, move the buffer
        Stream(Stream&& other) noexcept : parent_(other.parent_), oss_(std::move(other.oss_)) {}
        // Move assignment doesn't make sense with a reference member; delete it.
        Stream& operator=(Stream&&) = delete;

        template <typename T>
        Stream& operator<<(const T& v) { oss_ << v; return *this; }

        using Manip = std::ostream& (*)(std::ostream&);
        Stream& operator<<(Manip m) {
            if (m == static_cast<Manip>(std::endl<char, std::char_traits<char>>)) {
                m(oss_);
                flushBuffer(true);
            } else if (m == static_cast<Manip>(std::flush)) {
                flushBuffer(false);
            } else {
                m(oss_);
            }
            return *this;
        }

        ~Stream() {
            if (oss_.tellp() > 0) flushBuffer(false);
        }

    private:
        void flushBuffer(bool already_newlined) {
            auto s = oss_.str();
            if (!s.empty()) {
                if (already_newlined && s.back() == '\n') s.pop_back();
                parent_.log(s);
                oss_.str("");
                oss_.clear();
            }
        }

        CustomLogger& parent_;
        std::ostringstream oss_;
    };
};
