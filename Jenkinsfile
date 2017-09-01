#!groovy
node('xenial') {
  try {
    stage('Checkout') {
      sh 'mkdir -p catkin_ws/src/roscco'
      dir('catkin_ws/src/roscco')
      {
        checkout([
          $class: 'GitSCM',
          branches: scm.branches,
          doGenerateSubmoduleConfigurations: false,
          extensions: scm.extensions + [[$class: 'CleanBeforeCheckout'],
            [$class: 'SubmoduleOption',
            disableSubmodules: false,
            parentCredentials: true,
            recursiveSubmodules: true,
            reference: '',
            trackingSubmodules: false]],
          submoduleCfg: [],
          userRemoteConfigs: scm.userRemoteConfigs
        ])
      }
    }
    stage('Build') {
      parallel 'kia soul firmware': {
        sh '. /opt/ros/kinetic/setup.sh && cd catkin_ws && catkin_make -DKIA_SOUL=ON'
      }
      echo 'Build Complete!'
    }
    stage('Test') {
      parallel 'kia soul tests': {
        sh '. /opt/ros/kinetic/setup.sh && cd catkin_ws && catkin_make run_tests -DKIA_SOUL=ON'
        echo 'ROS Tests Complete!'
      }
    }
    stage('Release') {
      echo 'Release Package Created!'
    }
  }
  catch(Exception e) {
    throw e;
  }
  finally {
    deleteDir()
  }
}
