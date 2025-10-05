import { useMode } from "@/contexts/ModeContext";
import { useEffect, useState } from "react";

export const useFeatureFlag = (researcherOnly: boolean) => {
  const { mode } = useMode();
  const [isVisible, setIsVisible] = useState(!researcherOnly);

  useEffect(() => {
    setIsVisible(mode === "researcher" ? true : !researcherOnly);
  }, [mode, researcherOnly]);

  return isVisible;
};

export const useExplorerMode = () => {
  const { mode } = useMode();
  return mode === "explorer";
};

export const useResearcherMode = () => {
  const { mode } = useMode();
  return mode === "researcher";
};