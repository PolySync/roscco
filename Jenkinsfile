#!groovy

node() {
    def builds = [:]
    def platforms = [:]

    sh 'mkdir -p catkin_ws/src/roscco'
    dir('catkin_ws/src/roscco') {
        checkout scm
        sh "git submodule update --init --recursive"

        def image = docker.build("catkin_make-build:${env.BUILD_ID}")


        def output = image.inside {
            sh returnStdout: true, script: "cmake -LA ./oscc/firmware | grep 'VEHICLE_VALUES' | cut -d'=' -f 2"
        }

        platforms = output.trim().tokenize(';')\
    }

    for(int j=0; j<platforms.size(); j++) {
        def platform_idx = j
        def platform = platforms[platform_idx]
        builds[platform] = {
            node {
                sh 'mkdir -p catkin_ws/src/roscco'
                dir('catkin_ws/src/roscco') {
                    checkout scm
                    sh "git submodule update --init --recursive"
                }

                image = docker.build("catkin_make-build:${env.BUILD_ID}", "./catkin_ws/src/roscco")

                stage("Build ${platform}"){
                    image.inside {
                        sh ". /opt/ros/kinetic/setup.sh && \
                            cd catkin_ws && \
                            catkin_make -DVEHICLE=${platform}"

                        echo "${platform}: Build Complete!"
                    }
                }

                def workspace = pwd()

                stage("Test ${platform}"){
                    image.inside{
                        sh ". /opt/ros/kinetic/setup.sh && \
                            cd catkin_ws && \
                            ROS_HOME=${workspace} ROS_LOG_DIR=${workspace} catkin_make run_tests -DVEHICLE=${platform} && \
                            catkin_test_results --verbose"
                    }
                }
            }
        }
    }

    try {
        parallel builds
    }
    finally {
        deleteDir()
    }

}
