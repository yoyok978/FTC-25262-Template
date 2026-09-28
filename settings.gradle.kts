//pluginManagement {
//    repositories {
//        gradlePluginPortal()
//        mavenCentral()
//        google()
//        maven("https://repo.dairy.foundation/releases/") // <- This is required for the FTC plugin
//    }
//}
//
//include(":FtcRobotController")
//include(":TeamCode")

pluginManagement {
    repositories {
        gradlePluginPortal()
        mavenCentral()
        google()
        maven("https://repo.dairy.foundation/releases")
    }
}

plugins {
    id("org.gradle.toolchains.foojay-resolver-convention").version("1.0.0")
}