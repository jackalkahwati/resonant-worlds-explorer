import { useFeatureFlag } from "@/hooks/useFeatureFlag";

interface ResearcherOnlyProps {
  children: React.ReactNode;
  fallback?: React.ReactNode;
}

export const ResearcherOnly = ({ children, fallback = null }: ResearcherOnlyProps) => {
  const isVisible = useFeatureFlag(true);
  return isVisible ? children : fallback;
};