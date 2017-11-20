#ifndef FILEOBSERVER_H
#define FILEOBSERVER_H

#include <sys/inotify.h>
#include <boost/filesystem.hpp>
#include <functional>
#include <thread>
#include <mutex>

namespace bfs = boost::filesystem;

/**
 * @enum FSEventType
 * @brief Represents the kind of event happening to a file.
 */
typedef enum
{
    CREATED = 0,
    MODIFIED = 1,
    DELETED = 2
} FSEventType;

/**
 * @struct FSEvent
 * @brief POD for transporting information about an event.
 */
struct FSEvent
{
    bfs::path file; ///< path to the affected file
    FSEventType type; ///< wvent kind
};

// Function type for callback
typedef std::function<void(const FSEvent&)> EventCallback;

/**
 * @class FileObserver
 * @brief A Class that observes a filesystem path and registers callbacks which
 * are invoked for the specified event (create, modify, delete)
 * @note This is LLinux only
 */
class FileObserver
{
    private:
        bfs::path path;             ///< path to observe
        EventCallback created_cb;   ///< callback for file creation
        EventCallback modified_cb;  ///< callback for file contents modification (not `touch`)
        EventCallback deleted_cb;   ///< callback for file deletion
        std::thread runner;         ///< background thread polling inotify for file events
        std::mutex mutex;           ///< mutex for supervision read/write to callback members
        bool error;                 ///< error flag set by the thread when it can't do its work

        /**
         * @brief Oberve the path for file system events. Executed by the runner
         * thread
         */
        void observe();

    public:
		virtual ~FileObserver()
        {
            // TODO: Stop runner thread?
        }

        /**
         * @brief Create a new FileObserver. This will null-initialise all
         * callbacks. The runner thread starts immediately.
         */
        FileObserver(const bfs::path& path) :
            path(path),
            created_cb(nullptr),
            modified_cb(nullptr),
            deleted_cb(nullptr),
            runner(std::bind(&FileObserver::observe, this)),
            error(false)
        {
            runner.detach();
        }

        /**
         * @brief Register function to be called when a file is created.
         * @param callback an `std::function`, can be the result of `std::bind`
         */
        void on_created(EventCallback callback);

        /**
         * @brief Register function to be called when a file is modified.
         * @param callback an `std::function`, can be the result of `std::bind`
         */
        void on_modified(EventCallback callback);

        /**
         * @brief Register function to be called when a file is deleted.
         * @param callback an `std::function`, can be the result of `std::bind`
         */
        void on_deleted(EventCallback callback);

        /**
         * @brief Check whether thread is still alive
         */
        bool good();
};

#endif /* FILEOBSERVER_H */
