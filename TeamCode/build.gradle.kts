plugins {
    id("dev.frozenmilk.teamcode") version "11.1.0-1.1.1"
    id("dev.frozenmilk.sinister.sloth.load") version "0.2.4"
}

repositories {
    maven("https://mymaven.bylazar.com/releases")
    maven("https://repo.dairy.foundation/releases")
}

configure<com.android.build.gradle.AppExtension> {
    namespace = "org.firstinspires.ftc.teamcode"
    packagingOptions { jniLibs.useLegacyPackaging = true }
}

ftc {
    sdk {
        TeamCode()

    }

    dairy {
        implementation(Sloth)
    }

    ftControl {
        implementation(fullpanels)
    }

    pedro {
        implementation(core)
        implementation(ftc)
        implementation(telemetry)
    }

    solvers {
        implementation(core)
        implementation(pedroPathing)
    }
}

dependencies {
    "implementation"(project(":FtcRobotController"))
}