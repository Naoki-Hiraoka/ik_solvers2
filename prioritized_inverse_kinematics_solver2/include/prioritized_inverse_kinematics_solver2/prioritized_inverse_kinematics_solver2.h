#ifndef PRIORITIZED_INVERSE_KINEMATICS_SOLVER2_H
#define PRIORITIZED_INVERSE_KINEMATICS_SOLVER2_H

#include <cnoid/Body>
#include <ik_constraint2/IKConstraint.h>
#include <prioritized_qp_base/PrioritizedQPBaseSolver.h>
#include <prioritized_qp_osqp/prioritized_qp_osqp.h>

namespace prioritized_inverse_kinematics_solver2 {
  /*
    variables: 動かして良いjoint (free jointは6DOF扱い)
    ikc_list: タスクたち. vectorの前の要素の方が高優先度. 0番目の要素は必ず満たすと仮定しQPを解かない
    prevTasks: 前回のtasksを入れる. 自動的に更新される.

    返り値: 各constraintを満たしているかどうか
   */
  class IKParam {
  public:
    /*
     終了条件:
       maxIteration
       minIteraiion
       convertThre
       isSatisfied

     or maxIteraion
     or minIteration and isSatisfied
     or convergeThre and isSatisfied(if satisfiedConvergeLevel)
     */

    size_t maxIteration = 1; // constraintをsatisfiedするか、maxIteraionに達すると終了する.
    size_t minIteration = 0; // このiterationまでは、constraintをsatisfiedしても終了しない
    std::vector<double> dqWeight; // dqWeight.size() == dimの場合、探索変数の各要素について、wn+weをdqWeight倍する.
    double wn = 1e-6;
    std::vector<double> wnVec; // wnVec.size() == ikc_list.size()の場合、wnの代わりにこっちを使う
    double we = 1e0;
    std::vector<double> weVec; // weVec.size() == ikc_list.size()の場合、weの代わりにこっちを使う
    double wmax = 1e-1;
    std::vector<double> wmaxVec; // wmaxVec.size() == ikc_list.size()の場合、wmaxの代わりにこっちを使う
    int debugLevel = 0; // 0: no debug message. 1: time measure. 2: internal state

    double dt = 0.1;
    bool calcVelocity = true; // dtを用いて速度の計算をするかどうか. 速度を利用するconstraintがあるなら必須. ないなら、falseにすると高速化が見込まれる
    bool checkFinalState = true; // maxIteration番目またはconvergedのloop後に、各constraintを満たしているかどうかの判定を行うかどうか. 行わない場合、falseが返る.
    double convergeThre = 5e-3; // 各イテレーションでの変位のノルムがconvergeThre未満の場合に、maxIterationに行っていなくても, minIteraionに行っていなくても、isSatisfiedでなくても、終了する
    int satisfiedConvergeLevel = -1; // convergeThreを満たしても、ikclistのsatisfiedConvergeLevel番目の要素までがisSatisfiedでなければ終了しない.
    size_t pathOutputLoop = 1; // このloop回数に一回、途中経過のpathを出力する. 1以上

  };
  bool solveIKLoop (const std::vector<cnoid::LinkPtr>& variables,
                    const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& ikc_list,
                    std::vector<std::shared_ptr<prioritized_qp_base::Task> >& prevTasks,
                    const IKParam& param = IKParam(),
                    std::shared_ptr<std::vector<std::vector<double> > > path = nullptr, // 各イテレーションでの値. freejointはx y z qx qy qz qwの順. 始点と終点を含む
                    std::function<void(std::shared_ptr<prioritized_qp_base::Task>&,int)> taskGeneratorFunc = [](std::shared_ptr<prioritized_qp_base::Task>& task, int debugLevel){
                      std::shared_ptr<prioritized_qp_osqp::Task> taskOSQP = std::dynamic_pointer_cast<prioritized_qp_osqp::Task>(task);
                      if(!taskOSQP){
                        task = std::make_shared<prioritized_qp_osqp::Task>();
                        taskOSQP = std::dynamic_pointer_cast<prioritized_qp_osqp::Task>(task);
                      }
                      taskOSQP->settings().verbose = (debugLevel > 1);
                      taskOSQP->settings().max_iter = 4000;
                      taskOSQP->settings().eps_abs = 1e-3;// 大きい方が速いが，不正確. 1e-5はかなり小さい. 1e-4は普通
                      taskOSQP->settings().eps_rel = 1e-3;// 大きい方が速いが，不正確. 1e-5はかなり小さい. 1e-4は普通
                      taskOSQP->settings().scaled_termination = true;// avoid too severe termination check
                    }
                    );
  bool solveIKLoop (const std::vector<cnoid::LinkPtr>& variables,
                    const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& ikc_list,
                    const std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& rejections, // これをsatisfyしなくなる直前のstateを返す
                    std::vector<std::shared_ptr<prioritized_qp_base::Task> >& prevTasks,
                    const IKParam& param = IKParam(),
                    std::shared_ptr<std::vector<std::vector<double> > > path = nullptr, // 各イテレーションでの値. freejointはx y z qx qy qz qwの順. 始点と終点を含む
                    std::function<void(std::shared_ptr<prioritized_qp_base::Task>&,int)> taskGeneratorFunc = [](std::shared_ptr<prioritized_qp_base::Task>& task, int debugLevel){
                      std::shared_ptr<prioritized_qp_osqp::Task> taskOSQP = std::dynamic_pointer_cast<prioritized_qp_osqp::Task>(task);
                      if(!taskOSQP){
                        task = std::make_shared<prioritized_qp_osqp::Task>();
                        taskOSQP = std::dynamic_pointer_cast<prioritized_qp_osqp::Task>(task);
                      }
                      taskOSQP->settings().verbose = (debugLevel > 1);
                      taskOSQP->settings().max_iter = 4000;
                      taskOSQP->settings().eps_abs = 1e-3;// 大きい方が速いが，不正確. 1e-5はかなり小さい. 1e-4は普通
                      taskOSQP->settings().eps_rel = 1e-3;// 大きい方が速いが，不正確. 1e-5はかなり小さい. 1e-4は普通
                      taskOSQP->settings().scaled_termination = true;// avoid too severe termination check
                    }
                    );

}

#endif
