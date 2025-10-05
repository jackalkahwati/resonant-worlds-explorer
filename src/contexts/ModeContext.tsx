import { createContext, useContext, useEffect, useState } from "react";

type Mode = "explorer" | "researcher";

interface ModeContextType {
  mode: Mode;
  setMode: (mode: Mode) => void;
}

const ModeContext = createContext<ModeContextType | undefined>(undefined);

export const ModeProvider = ({ children }: { children: React.ReactNode }) => {
  const [mode, setMode] = useState<Mode>(() => {
    const savedMode = localStorage.getItem("appMode");
    return (savedMode as Mode) || "explorer";
  });

  useEffect(() => {
    localStorage.setItem("appMode", mode);
    document.body.dataset.mode = mode;
  }, [mode]);

  return (
    <ModeContext.Provider value={{ mode, setMode }}>
      {children}
    </ModeContext.Provider>
  );
};

export const useMode = () => {
  const context = useContext(ModeContext);
  if (context === undefined) {
    throw new Error("useMode must be used within a ModeProvider");
  }
  return context;
};