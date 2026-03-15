pluginManagement {
    repositories {
        gradlePluginPortal()
        mavenCentral()
        google()
        maven("https://repo.dairy.foundation/releases/") // <- This is required for the FTC plugin
    }
}

include(":FtcRobotController")
include(":TeamCode")