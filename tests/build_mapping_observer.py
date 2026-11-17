#!/usr/bin/env python3
"""Build an observation-only mapping copy; never overwrite installed LIO binaries."""
import argparse
import difflib
import hashlib
import json
from pathlib import Path
import shlex
import subprocess


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--source', type=Path, default=Path('/home/yang/wzw/ct_ws/src/LIO-SAM'))
    parser.add_argument('--build', type=Path, default=Path('/home/yang/wzw/ct_ws/build/lio_sam'))
    parser.add_argument('--plane-mode', choices=['original','adaptive_pca'], default='original',
                        help='adaptive_pca is an unaccepted experiment in the separate copy only')
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    original = (args.source/'src/mapOptmization.cpp').read_text()
    text = original

    def replace(old, new):
        nonlocal text
        assert text.count(old) == 1, ('source does not match observation hook', old[:100])
        text = text.replace(old, new)

    replace('    bool scanOptimizationFailed = false;', '''    bool scanOptimizationFailed = false;
    std::ofstream diagnosticPose, diagnosticPlanes;
    std::vector<std::array<float, 29>> diagnosticSurfaceRows;
    std::vector<unsigned char> diagnosticSurfaceValid;
    float diagnosticPredicted[6], diagnosticOptimized[6], diagnosticFinalized[6];
    int diagnosticIteration = -1;''')
    replace('        ISAM2Params parameters;', '''        std::string diagnosticDirectory;
        ros::param::get("/diagnostic/output_directory", diagnosticDirectory);
        if (diagnosticDirectory.empty()) throw std::runtime_error("diagnostic output directory required");
        diagnosticPose.open(diagnosticDirectory + "/pose_stages.csv");
        diagnosticPlanes.open(diagnosticDirectory + "/surface_correspondences.csv");
        if (!diagnosticPose || !diagnosticPlanes) throw std::runtime_error("diagnostic output unavailable");
        diagnosticPose << std::setprecision(10);
        diagnosticPlanes << std::setprecision(10);
        diagnosticPose << "stamp,pred_roll,pred_pitch,pred_yaw,pred_x,pred_y,pred_z,opt_roll,opt_pitch,opt_yaw,opt_x,opt_y,opt_z,finalize_roll,finalize_pitch,finalize_yaw,finalize_x,finalize_y,finalize_z,graph_roll,graph_pitch,graph_yaw,graph_x,graph_y,graph_z,optimized,failed,degenerate,lambda_min,residual_initial,residual_final,iterations,keyframes" << std::endl;
        diagnosticPlanes << "stamp,ori_x,ori_y,ori_z,selected_x,selected_y,selected_z,pa,pb,pc,pd,residual,weight,n0_x,n0_y,n0_z,n1_x,n1_y,n1_z,n2_x,n2_y,n2_z,n3_x,n3_y,n3_z,n4_x,n4_y,n4_z,range5_sq,accepted" << std::endl;
        diagnosticSurfaceRows.resize(N_SCAN * Horizon_SCAN);
        diagnosticSurfaceValid.resize(N_SCAN * Horizon_SCAN, 0);
        ISAM2Params parameters;''')
    replace('            scan2MapOptimization();', '''            std::copy(transformTobeMapped, transformTobeMapped+6, diagnosticPredicted);
            scan2MapOptimization();
            std::copy(transformTobeMapped, transformTobeMapped+6, diagnosticOptimized);''')
    replace('            transformUpdate();', '''            transformUpdate();
            std::copy(transformTobeMapped, transformTobeMapped+6, diagnosticFinalized);''')
    replace('            publishOdometry();', '''            diagnosticPose << timeLaserInfoCur;
            for (const float value : diagnosticPredicted) diagnosticPose << ',' << value;
            for (const float value : diagnosticOptimized) diagnosticPose << ',' << value;
            for (const float value : diagnosticFinalized) diagnosticPose << ',' << value;
            for (const float value : transformTobeMapped) diagnosticPose << ',' << value;
            diagnosticPose << ',' << scanOptimizationPerformed << ',' << scanOptimizationFailed << ',' << isDegenerate
                << ',' << scan2MapDebugMsg.lambda_min << ',' << scan2MapDebugMsg.initial_residual_rmse
                << ',' << scan2MapDebugMsg.final_residual_rmse << ',' << int(scan2MapDebugMsg.iterations)
                << ',' << cloudKeyPoses3D->size() << std::endl;
            publishOdometry();''')
    replace('                    coeff.x = s * pa;', '''                    if (diagnosticIteration == 0 && ((timeLaserInfoCur >= 189 && timeLaserInfoCur <= 199) || timeLaserInfoCur < 180)) {
                        auto &row = diagnosticSurfaceRows[i];
                        row = {{pointOri.x, pointOri.y, pointOri.z, pointSel.x, pointSel.y, pointSel.z,
                            pa, pb, pc, pd, pd2, s, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, pointSearchSqDis[4], float(s>0.1)}};
                        for (int n=0; n<5; ++n) {
                            const auto &point = laserCloudSurfFromMapDS->points[pointSearchInd[n]];
                            row[12+n*3] = point.x; row[13+n*3] = point.y; row[14+n*3] = point.z;
                        }
                        diagnosticSurfaceValid[i] = 1;
                    }
                    coeff.x = s * pa;''')
    replace('                cornerOptimization();', '''                diagnosticIteration = iterCount;
                std::fill(diagnosticSurfaceValid.begin(), diagnosticSurfaceValid.end(), 0);
                cornerOptimization();''')
    replace('                combineOptimizationCoeffs();', '''                combineOptimizationCoeffs();
                if (iterCount == 0) {
                    for (int i=0; i<laserCloudSurfLastDSNum; ++i) {
                        if (!diagnosticSurfaceValid[i]) continue;
                        diagnosticPlanes << timeLaserInfoCur;
                        for (const float value : diagnosticSurfaceRows[i]) diagnosticPlanes << ',' << value;
                        diagnosticPlanes << '\\n';
                    }
                    diagnosticPlanes.flush();
                }''')
    if args.plane_mode == 'adaptive_pca':
        start = text.index('            Eigen::Matrix<float, 5, 3> matA0;', text.index('    void surfOptimization()'))
        end = text.index('                if (planeValid) {', start)
        # Keep a 1 m search bound and the original 0.2 m plane consistency
        # threshold. Require two supported tangent directions; a single ring
        # cannot define an arbitrary horizontal/vertical surface plane.
        text = text[:start] + '''            if (pointSearchSqDis[4] < 1.0) {
                kdtreeSurfFromMap->nearestKSearch(pointSel, 20, pointSearchInd, pointSearchSqDis);
                Eigen::Vector3f normal = Eigen::Vector3f::Zero();
                Eigen::Vector3f center = Eigen::Vector3f::Zero();
                bool planeValid = false;
                int support = 0;
                const int available = int(pointSearchInd.size());
                for (int count : {5, 10, 20}) {
                    if (count > available || pointSearchSqDis[count-1] >= 1.0) continue;
                    center.setZero();
                    for (int j=0; j<count; ++j) center += laserCloudSurfFromMapDS->points[pointSearchInd[j]].getVector3fMap();
                    center /= count;
                    Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
                    for (int j=0; j<count; ++j) {
                        Eigen::Vector3f delta = laserCloudSurfFromMapDS->points[pointSearchInd[j]].getVector3fMap()-center;
                        covariance += delta*delta.transpose();
                    }
                    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance/count);
                    if (solver.info()!=Eigen::Success) continue;
                    auto e=solver.eigenvalues();
                    if (!e.allFinite() || e[2]<=1e-8 || e[1]<0.01f*e[2] || e[0]>0.1f*e[1]) continue;
                    normal=solver.eigenvectors().col(0);
                    planeValid=true;
                    for (int j=0; j<count; ++j) {
                        if (std::abs(normal.dot(laserCloudSurfFromMapDS->points[pointSearchInd[j]].getVector3fMap()-center))>0.2f) {
                            planeValid=false; break;
                        }
                    }
                    if (planeValid) { support=count; break; }
                }
                if (!support) continue;
                float pa=normal.x(), pb=normal.y(), pc=normal.z(), pd=-normal.dot(center);

''' + text[end:]
    source = output/'mapping_observer.cpp'
    source.write_text(text)
    patch=''.join(difflib.unified_diff(original.splitlines(True),text.splitlines(True),
        fromfile='original/mapOptmization.cpp',tofile='diagnostic/mapping_observer.cpp'))
    (output/'mapping_observer.patch').write_text(patch)
    flags = {}
    for line in (args.build/'CMakeFiles/lio_sam_mapOptmization.dir/flags.make').read_text().splitlines():
        if ' = ' in line:
            key, value = line.split(' = ', 1)
            flags[key] = shlex.split(value)
    # Debug symbols are unnecessary in this separate diagnostic build.
    compile_flags = [x for x in flags['CXX_FLAGS'] if x != '-g']
    obj = output/'mapping_observer.o'
    binary = output/'mapping_observer'
    compile_command = ['/usr/bin/c++'] + compile_flags + flags['CXX_DEFINES'] + flags['CXX_INCLUDES'] + ['-c', str(source), '-o', str(obj)]
    link_command = shlex.split((args.build/'CMakeFiles/lio_sam_mapOptmization.dir/link.txt').read_text())
    link_command[link_command.index('-o')+1] = str(binary)
    link_command = [str(obj) if x.endswith('mapOptmization.cpp.o') else x for x in link_command if x != '-g']
    manifest = {'purpose':('observation-only copy; algorithm decisions unchanged' if args.plane_mode=='original'
                          else 'unaccepted adaptive PCA plane experiment in separate copy')+'; production source/binary untouched',
        'plane_mode':args.plane_mode,
        'source_sha256':hashlib.sha256(original.encode()).hexdigest(),
        'observer_sha256':hashlib.sha256(text.encode()).hexdigest(),
        'patch_sha256':hashlib.sha256(patch.encode()).hexdigest(),
        'compile_command':compile_command, 'link_command':link_command}
    (output/'build.json').write_text(json.dumps(manifest, indent=2)+'\n')
    subprocess.run(compile_command, check=True)
    subprocess.run(link_command, check=True)
    manifest['binary_sha256'] = hashlib.sha256(binary.read_bytes()).hexdigest()
    (output/'build.json').write_text(json.dumps(manifest, indent=2)+'\n')
    print(binary, flush=True)


if __name__ == '__main__':
    main()
