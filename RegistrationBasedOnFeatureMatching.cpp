RegistrationResult RegistrationRANSACBasedOnFeatureMatching(
        const PointCloud &source,
        const PointCloud &target,
        const Feature &source_feature,
        const Feature &target_feature,
        double max_correspondence_distance,
        const TransformationEstimation &estimation
        /* = TransformationEstimationPointToPoint(false)*/,
        const std::vector<std::reference_wrapper<const CorrespondenceChecker>>
                &checkers /* = {}*/) {
    if (max_correspondence_distance <= 0.0) {
        return RegistrationResult();
    }

    RegistrationResult result;
    int num_similar_features = 1;
    std::vector<std::vector<int>> similar_features(source.points_.size());
    int source_size = source.points_.size();

#ifdef _OPENMP
#pragma omp parallel
    {
#endif
        CorrespondenceSet corres(source_size);
        KDTreeFlann kdtree(target); // ok
        KDTreeFlann kdtree_feature(target_feature); // ok 
        RegistrationResult result_private; // ok

#ifdef _OPENMP
#pragma omp for nowait
#endif
        std::vector<double> dists(num_similar_features);
        Eigen::Matrix4d transformation;

        for (int itr = 0; itr < source_size; itr++) {
            source_sample_id = itr;
            if (similar_features[source_sample_id].empty()) {
                std::vector<int> indices(num_similar_features);
                kdtree_feature.SearchKNN(
                                Eigen::VectorXd(source_feature.data_.col(source_sample_id)),
                                num_similar_features, 
                                indices, dists);
#ifdef _OPENMP
#pragma omp critical
#endif
                { similar_features[source_sample_id] = indices; }
            } // end if
            
            corres[j](0) = source_sample_id;
            if (num_similar_features == 1)
                corres[j](1) =
                        similar_features[source_sample_id][0]; // indices is like [n], a vector
            else
                corres[j](1) =
                        similar_features[source_sample_id]
                                        [std::rand() %
                                         num_similar_features]; // pick a random indice between the NNs

        } // end for


                









                bool check = true;
                for (const auto &checker : checkers) {
                    if (checker.get().require_pointcloud_alignment_ == false &&
                        checker.get().Check(source, target, ransac_corres,
                                            transformation) == false) {
                        check = false;
                        break; // if a checker that does not require transformation fails, no need to continue
                               // that ransac cycle fails
                    }
                }
                if (check == false) continue; // that ransac cycle fail 
                // if not, estimate transformation to allow the checker which need transformed cloud
                transformation = estimation.ComputeTransformation(
                        source, target, ransac_corres);
                check = true;
                for (const auto &checker : checkers) {
                    if (checker.get().require_pointcloud_alignment_ == true &&
                        checker.get().Check(source, target, ransac_corres,
                                            transformation) == false) {
                        check = false;
                        break; // if one of these checker(s) failed, that ransac cycle fails
                    } 
                }
                if (check == false) continue; 
                PointCloud pcd = source;
                pcd.Transform(transformation);
                auto this_result = GetRegistrationResultAndCorrespondences(
                        pcd, target, kdtree, max_correspondence_distance,
                        transformation);
                if (this_result.fitness_ > result_private.fitness_ ||
                    (this_result.fitness_ == result_private.fitness_ &&
                     this_result.inlier_rmse_ < result_private.inlier_rmse_)) {
                    result_private = this_result;
                }
#ifdef _OPENMP
#pragma omp critical
#endif
                {
                    total_validation = total_validation + 1;
                    if (total_validation >= criteria.max_validation_)
                        finished_validation = true;
                }
            }  // end of if statement
        }      // end of for-loop
#ifdef _OPENMP
#pragma omp critical
#endif
        {
            if (result_private.fitness_ > result.fitness_ ||
                (result_private.fitness_ == result.fitness_ &&
                 result_private.inlier_rmse_ < result.inlier_rmse_)) {
                result = result_private;
            }
        }
#ifdef _OPENMP
    }
#endif
    PrintDebug("total_validation : %d\n", total_validation);
    PrintDebug("RANSAC: Fitness %.4f, RMSE %.4f\n", result.fitness_,
               result.inlier_rmse_);
    return result;
}