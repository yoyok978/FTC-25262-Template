plugins {
    id("dev.frozenmilk.teamcode") version "11.1.0-1.1.1"
    id("dev.frozenmilk.sinister.sloth.load") version "0.2.4"
}

repositories {
    maven("https://repo.dairy.foundation/releases")
}

configure<com.android.build.gradle.AppExtension> {
    namespace = "org.firstinspires.ftc.teamcode"
    compileSdkVersion(34)
    packagingOptions { jniLibs.useLegacyPackaging = true }
}

ftc {
    // 1. Core SDK
    sdk.TeamCode()

    // 2. Sloth
    dairy {
        implementation(Sloth)
    }

    // 3. Panels
    ftControl {
        implementation(fullpanels)
    }

    // 4. Pedro Pathing
    pedro {
        implementation(core)
        implementation(ftc)
        implementation(telemetry)
    }

    // 5. SolversLib
    solvers {
        implementation(core)
        implementation(pedroPathing)
    }
}

dependencies {
    "implementation"(project(":FtcRobotController"))
}